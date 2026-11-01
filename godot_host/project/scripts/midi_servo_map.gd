extends RefCounted
## (loaded by preload("res://scripts/midi_servo_map.gd"); no class_name, so headless runs need no global class cache)
## A MIDI controller's sliders and knobs as servo controls, for pose and toe experiments on the
## bench (operator, 2026-11-01: "a more tactile way of controlling the servos manually").
##
## A LOCAL EXPERIMENTAL TOOL, used only while the brain is NOT commanding the servos: the daemon
## refuses servo.set in a brain mode anyway (SPEC §1.1).  A mapped control moves its row's slider,
## and the slider moves the servo (no ARM step, 2026-11-01).  The map lives in its own file, outside the brain's configs:
##   res://local/midi_map.json         (git-ignored; written by LEARN)
##   res://midi_map.example.json       (the AKAI MIDI Mix defaults, committed)
##
## MAP.  {"device": "...", "cc": {"<controller>": {"ch": <servo 0..11>, "invert": <bool>}}, "notes": {"<note>": "<action>"}}
## (a bare integer value is read as {"ch": n, "invert": false}).  INVERT flips a control for a servo
## whose joint runs the other way from the knob (hip1 on this body), per servo, saved with the map.
## A CC's 0..127 becomes t in [0, 1]; the OWNER maps t onto that servo's own limits (the dash: its
## row's min/max µs, which LOAD MAP fills from pi_host/calib/servo_map.json; the sim panel: its
## slider range).  Actions: "stop" (the dash's STOP).
##
## PICKUP.  A fader that does not match the servo it is mapped to must not yank it: a control takes
## hold only once its value has crossed, or come within `pickup_window` of, the servo's CURRENT
## normalised position (the owner's `current_cb`).  Until then its moves are ignored and reported.
##
## LEARN.  Set `learn_ch` (or `learn_action`) and move a control / press a button: the first CC
## (note) seen is bound, the map saved, `learned` emitted.

signal control(ch: int, t: float)          # a mapped CC moved and has pickup: servo channel, t in [0,1]
signal action(name: String)                # a mapped note was pressed
signal learned(kind: String, number: int, target: String)
signal status(msg: String)

const LOCAL_PATH := "res://local/midi_map.json"
const EXAMPLE_PATH := "res://midi_map.example.json"
const MIDI_CC := 0xB
const MIDI_NOTE_ON := 0x9

var map: Dictionary = {"device": "", "cc": {}, "notes": {}}
var map_path: String = ""
var learn_ch: int = -1
var learn_action: String = ""
var pickup: bool = true
var pickup_window: float = 0.04
var current_cb: Callable = Callable()      # (ch) -> float t in [0,1], or -1.0 when unknown
var devices: PackedStringArray = PackedStringArray()

var _caught: Dictionary = {}               # cc -> bool
var _opened: bool = false
var _last_t: Dictionary = {}               # cc -> float


## Opens the MIDI inputs once.  Never in a headless run: there is no input there, and Godot's
## MIDI driver aborts without a display server (seen 2026-11-01 on the headless smoke runs).
func open_inputs() -> PackedStringArray:
	if DisplayServer.get_name() == "headless" or OS.has_feature("headless"):
		devices = PackedStringArray(); return devices
	if not _opened:
		OS.open_midi_inputs(); _opened = true
	devices = OS.get_connected_midi_inputs()
	return devices


func load_map() -> String:
	for p in [LOCAL_PATH, EXAMPLE_PATH]:
		if FileAccess.file_exists(p):
			var f := FileAccess.open(p, FileAccess.READ)
			if f == null: continue
			var parsed: Variant = JSON.parse_string(f.get_as_text())
			if parsed is Dictionary:
				map = {"device": str(parsed.get("device", "")), "cc": {}, "notes": {}}
				for k in (parsed.get("cc", {}) as Dictionary).keys():
					var e = (parsed["cc"] as Dictionary)[k]
					map["cc"][str(k)] = {"ch": int(e["ch"]), "invert": bool(e.get("invert", false))} if e is Dictionary else {"ch": int(e), "invert": false}
				for k in (parsed.get("notes", {}) as Dictionary).keys(): map["notes"][str(k)] = str((parsed["notes"] as Dictionary)[k])
				map_path = p
				_caught.clear(); _last_t.clear()
				return p
	map_path = ""
	return ""


func save_map() -> bool:
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path("res://local"))
	var f := FileAccess.open(LOCAL_PATH, FileAccess.WRITE)
	if f == null: return false
	var out := {"_comment": "MIDI -> servo map for the bench dashboard and the sim servo panel (MidiServoMap). cc: controller number -> {ch: servo channel P0..P11, invert: flip the control for that servo}; notes: note number -> action (stop). Written by LEARN and the inv toggles; local, not a brain config.",
		"device": map.get("device", ""), "cc": map["cc"], "notes": map["notes"]}
	f.store_string(JSON.stringify(out, "  ")); f.close()
	map_path = LOCAL_PATH
	return true


func binding_for(ch: int) -> String:
	for k in (map["cc"] as Dictionary).keys():
		if int(map["cc"][k]["ch"]) == ch: return "cc%s%s" % [k, " inv" if bool(map["cc"][k]["invert"]) else ""]
	return ""


func invert_for(ch: int) -> bool:
	for k in (map["cc"] as Dictionary).keys():
		if int(map["cc"][k]["ch"]) == ch: return bool(map["cc"][k]["invert"])
	return false


## Flip (or not) the control bound to a servo; saved at once.  Returns false when nothing is bound.
func set_invert(ch: int, on: bool) -> bool:
	var found := false
	for k in (map["cc"] as Dictionary).keys():
		if int(map["cc"][k]["ch"]) == ch:
			map["cc"][k]["invert"] = on; found = true
			_caught.erase(str(k))             # the control must find the servo again in its new sense
	if found: save_map()
	return found


func note_for(act: String) -> String:
	for k in (map["notes"] as Dictionary).keys():
		if str(map["notes"][k]) == act: return "note %s" % k
	return ""


## Forget a control's pickup (the servo moved by other means, or LIVE was just switched on).
func release_pickup_all() -> void:
	_caught.clear()


## Feed every InputEventMIDI here.  Returns a one-line status ("" = nothing to say).
func handle(ev: InputEventMIDI) -> String:
	if ev.message == MIDI_CC:
		var cc := int(ev.controller_number)
		var t: float = clampf(float(ev.controller_value) / 127.0, 0.0, 1.0)
		if learn_ch >= 0:
			# One control per servo: drop any older binding of this CC and of this servo (the servo's
			# invert setting survives the re-binding).
			var inv := invert_for(learn_ch)
			for k in (map["cc"] as Dictionary).keys():
				if int(map["cc"][k]["ch"]) == learn_ch or int(k) == cc: (map["cc"] as Dictionary).erase(k)
			map["cc"][str(cc)] = {"ch": learn_ch, "invert": inv}
			var ch := learn_ch; learn_ch = -1
			_caught.erase(str(cc))
			save_map()
			learned.emit("cc", cc, "P%d" % ch)
			return "MIDI learned: cc%d -> P%d (saved %s)" % [cc, ch, LOCAL_PATH]
		var key := str(cc)
		if not (map["cc"] as Dictionary).has(key): return ""
		var ch2 := int(map["cc"][key]["ch"])
		if bool(map["cc"][key]["invert"]): t = 1.0 - t
		if pickup and not bool(_caught.get(key, false)):
			var cur: float = current_cb.call(ch2) if current_cb.is_valid() else -1.0
			var last: float = float(_last_t.get(key, -1.0))
			_last_t[key] = t
			var ok := cur < 0.0 or absf(t - cur) <= pickup_window or (last >= 0.0 and (last - cur) * (t - cur) <= 0.0)
			if not ok:
				return "MIDI cc%d -> P%d waiting for pickup (control %.2f, servo %.2f)" % [cc, ch2, t, cur]
			_caught[key] = true
		_last_t[key] = t
		control.emit(ch2, t)
		return ""
	if ev.message == MIDI_NOTE_ON and ev.velocity > 0:
		var note := int(ev.pitch)
		if learn_action != "":
			for k in (map["notes"] as Dictionary).keys():
				if str(map["notes"][k]) == learn_action or int(k) == note: (map["notes"] as Dictionary).erase(k)
			map["notes"][str(note)] = learn_action
			var act := learn_action; learn_action = ""
			save_map()
			learned.emit("note", note, act)
			return "MIDI learned: note %d -> %s (saved %s)" % [note, act, LOCAL_PATH]
		var k2 := str(note)
		if (map["notes"] as Dictionary).has(k2):
			action.emit(str(map["notes"][k2]))
	return ""


func describe() -> String:
	var dev := ", ".join(devices) if devices.size() > 0 else "no MIDI input"
	var n := (map["cc"] as Dictionary).size()
	return "%s   %d control%s mapped%s" % [dev, n, "" if n == 1 else "s", ("   (%s)" % map_path.get_file()) if map_path != "" else "   (no map file)"]
