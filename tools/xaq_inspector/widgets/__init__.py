"""Per-module inspector widgets.

Dispatch table maps the C++ module type_name string (as returned by the
control server's `list_modules` verb) to a widget class.  Add a new
module type by importing its widget here and adding a row.
"""
from typing import Type

from PyQt6.QtWidgets import QWidget

from ._description_panel import wrap_with_description
from .raw_payload     import RawPayloadView
from .epm_canvas       import EpmCanvas
from .epm_inspector    import EpmInspector
from .neuro_inspector  import NeuroInspector
from .drive_inspector  import DriveInspector
from .fader_inspector  import FaderInspector
from .voter_inspector    import VoterInspector
from .premotor_inspector import PremotorInspector
from .reflex_inspector   import ReflexInspector
from .seqgng_inspector   import SeqGNGInspector
from .cpg_inspector      import CPGInspector
from .cruse_inspector    import CruseInspector
from .scent_compass_inspector import ScentCompassInspector
from .motor_epm_inspector     import MotorEpmInspector
from .action_plan_inspector   import ActionPlanInspector
from .motor_bus_inspector     import MotorBusInspector
from .heading_controller_inspector import HeadingControllerInspector
from .place_graph_inspector        import PlaceGraphInspector
from .place_nav_inspector          import PlaceNavInspector
from .play_loop_inspector          import PlayLoopInspector
from .klino_inspector              import KlinotaxisInspector
from .run_tumble_inspector         import RunTumbleInspector
from .run_tumble_v2_inspector      import RunTumbleV2Inspector
from .cylinder_inspector           import CylinderInspector
from .efe_arbiter_inspector        import EFEArbiterInspector
from .vision_bearing_inspector     import VisualBearingInspector
from .vision_homing_inspector      import VisualHomingInspector
from .piano_roll_inspector         import PianoRollInspector
from .gain_evolver_inspector       import GainEvolverInspector


WIDGET_REGISTRY: dict[str, Type[QWidget]] = {
    "EPM":                  EpmInspector,
    "NeurochemState":       NeuroInspector,
    "HomeostaticDrive":     DriveInspector,
    "FaderController":      FaderInspector,
    "LateralVoter":         VoterInspector,
    "Premotor":             PremotorInspector,
    "SequenceGNG":          SeqGNGInspector,
    "CPGOscillator":        CPGInspector,
    "CruseCoordinator":     CruseInspector,
    # The Cell's opaque cognitive trio (2026-06-19) — perception bearing,
    # homeokinetic self-model, and the coxswain's plan.
    "ScentCompass":         ScentCompassInspector,
    "MotorEPM":             MotorEpmInspector,
    # MotorEPMv2 began as a byte-identical copy of MotorEPM (the differ gate) and
    # still emits the same FLAT diag_snapshot keys — motor_tle, fwd_v, fwd_progress_ema,
    # commit_prec, gait_coherence, ... — so it reuses the same dashboard rather than
    # forking one.  Without this line a v2 module shows NO panel at all, which is why
    # the picrawler's fwd_v went unreadable the moment the stack moved to v2.
    "MotorEPMv2":           MotorEpmInspector,
    "ActionDecoder":        ActionPlanInspector,
    "MotorBus":             MotorBusInspector,
    "HeadingController":    HeadingControllerInspector,
    "PlaceGraphPlanner":    PlaceGraphInspector,
    # The planner reframed (2026-07-09) — place/region NAVIGATOR + loose honest food tag.
    "PlaceNav":             PlaceNavInspector,
    # The Cell's third policy (task #33) — GROW the shared map (epistemic play):
    # mirrors the planner dashboard with novelty-field + CLIMB↔WANDER semantics.
    "PlayLoop":             PlayLoopInspector,
    "Klinotaxis":           KlinotaxisInspector,
    # The maze klino (run-and-tumble) + place-code panorama builder (2026-06-29).
    "RunTumbleNav":         RunTumbleInspector,
    # The clean-room KF-ladder taxis (2026-07-08) — reuses the run-tumble panels + a KF readout.
    "RunTumbleNavV2":       RunTumbleV2Inspector,
    "CylinderBuilder":      CylinderInspector,
    # The Cell's L2 active-inference policy selector (2026-06-29) — the value race +
    # winner-take-all gain faders (klino CLOSER vs planner SEARCHER).
    "EFEArbiter":           EFEArbiterInspector,
    # The Cell's 4th loop — scent-independent visual food channel (2026-07-11):
    # VisualBearing (food-pixel bearing) → VisualHomingNav (close on a SEEN source).
    # Food is FOV-gated so both read 0 until food is in view (status banners make that legible).
    "VisualBearing":        VisualBearingInspector,
    "VisualHomingNav":      VisualHomingInspector,
    # PART III (2026-08-11) — the motor piano roll: the planner's probability cone
    # decoded to per-joint timelines, with the EARNED authority horizon drawn as
    # a gold line (verified cone accuracy vs the persistence baseline, per depth).
    "MotorPlanner":         PianoRollInspector,
    # PART IV (2026-08-17) — the lifetime (1+1)-ES over the gain vector: the
    # bounded-range rack (where each gain sits NOW, incumbent vs candidate),
    # the normalized trajectory, the criterion J, and the weighted term
    # breakdown that shows whether a term is dead.
    "GainEvolver":          GainEvolverInspector,
    # Generic reflex / detector widget — auto-discovers fields from the
    # snapshot, so one ReflexInspector suffices for every reflex type
    # without per-type bespoke panels.
    "CellReflex":           ReflexInspector,
    "WhiskerSteerReflex":   ReflexInspector,
    "WhiskerAversionReflex":ReflexInspector,
    "ScentGateReflex":      ReflexInspector,
    "StuckEscapeReflex":    ReflexInspector,
    "ForwardDriveReflex":   ReflexInspector,
    "DualEMADetector":      ReflexInspector,
    "AdaptiveThresholdTracker": ReflexInspector,
    "MotorFader":           ReflexInspector,
    "ActionGate":           ReflexInspector,
}

DEFAULT_WIDGET: Type[QWidget] = RawPayloadView


def widget_for(module_type: str) -> Type[QWidget]:
    return WIDGET_REGISTRY.get(module_type, DEFAULT_WIDGET)
