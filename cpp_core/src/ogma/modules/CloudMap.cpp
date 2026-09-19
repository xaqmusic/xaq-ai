#include "ogma/modules/CloudMap.hpp"

#include <algorithm>
#include <cmath>

#include <nlohmann/json.hpp>

namespace ogma {
namespace {
constexpr double kPi = 3.14159265358979323846;

double get_d(ParamMap const& p, const char* k, double dflt) {
    auto it = p.find(k);
    if (it == p.end()) return dflt;
    if (auto d = std::get_if<double>(&it->second)) return *d;
    if (auto i = std::get_if<int64_t>(&it->second)) return double(*i);
    if (auto b = std::get_if<bool>(&it->second)) return *b ? 1.0 : 0.0;
    return dflt;
}
std::string get_s(ParamMap const& p, const char* k, const char* dflt = "") {
    auto it = p.find(k);
    if (it == p.end()) return dflt;
    if (auto s = std::get_if<std::string>(&it->second)) return *s;
    return dflt;
}
}  // namespace

std::string_view CloudMap::type_name() const { return "CloudMap"; }

std::vector<TopicSpec> CloudMap::input_topics() const {
    std::vector<TopicSpec> t;
    if (!input_topic_.empty())
        t.emplace_back(input_topic_, std::type_index(typeid(ProprioToken)), SubscriptionKind::Direct, false);
    if (!place_topic_.empty())
        t.emplace_back(place_topic_, std::type_index(typeid(RealityToken)), SubscriptionKind::Direct, false);
    return t;
}

std::vector<TopicSpec> CloudMap::output_topics() const {
    std::vector<TopicSpec> t;
    if (!output_topic_.empty()) t.emplace_back(output_topic_, std::type_index(typeid(ProprioToken)));
    if (!change_topic_.empty()) t.emplace_back(change_topic_, std::type_index(typeid(ProprioToken)));
    if (!things_topic_.empty()) t.emplace_back(things_topic_, std::type_index(typeid(ProprioToken)));
    if (!thing_bearing_topic_.empty()) t.emplace_back(thing_bearing_topic_, std::type_index(typeid(ProprioToken)));
    return t;
}

ParamSchema CloudMap::params_schema() const {
    return {
        {"input_topic", ParamMutability::ConstructionOnly,
         "ProprioToken carrying one ToF cast in the GRAVITY-LEVELLED body frame: "
         "[still, yaw, trunk_z, odom_x, odom_y, then kZones x (x, y, z)] where z is height above "
         "the floor and a NaN x marks a zone that returned nothing.  `still` is the body's own "
         "stillness flag, `yaw` the heading the de-rotation is taken against, and odom_x/odom_y the "
         "dead-reckoned position the cache needs to line two visits up.  All of it the body's own "
         "(contact odometry, IMU).  Empty = the module is inert and the graph is byte-identical.",
         ParamValue{std::string("")}},
        {"place_topic", ParamMutability::ConstructionOnly,
         "RealityToken whose winner_id keys the cache — the place, in the map's own vocabulary "
         "rather than a coordinate.  The key is the MODAL winner while the cloud was open, "
         "because a map flickers between adjacent nodes at a still gaze (measured: 23 switches "
         "a minute even standing).  Empty = no cache and no revisit_change.",
         ParamValue{std::string("")}},
        {"output_topic", ParamMutability::ConstructionOnly,
         "Where the cloud's FLOOR-BREAK PROFILE goes, as a ProprioToken of kProfile values: per "
         "azimuth sector the nearest break's range, its height, its vertical extent and its voxel "
         "mass, then four globals (break mass, azimuth span, mean height, cloud size).  All in "
         "[0,1].  An EPM on this topic earns the object vocabulary; this module does not.",
         ParamValue{std::string("")}},
        {"change_topic", ParamMutability::ConstructionOnly,
         "Optional ProprioToken [new_fraction, revisit_change]: change WITHIN this sweep (a mover "
         "crossing it) and change against this place's stored cloud (the room rearranged since we "
         "were last here).  revisit_change is -1 when no cloud is cached for this place.",
         ParamValue{std::string("")}},
        {"voxel_m", ParamMutability::HotMutable,
         "Voxel edge, metres.  0.04 makes one voxel about one block, which is the scale the "
         "sub-pixel measurement argues for.", ParamValue{0.04}, ParamValue{0.005}, ParamValue{0.5}},
        {"break_lo", ParamMutability::HotMutable,
         "A return this far above the floor is not the floor (metres).", ParamValue{0.02}, ParamValue{0.0}, ParamValue{1.0}},
        {"break_hi", ParamMutability::HotMutable,
         "...and this low is something standing ON the floor rather than furniture (metres).",
         ParamValue{0.20}, ParamValue{0.0}, ParamValue{2.0}},
        {"half_fov", ParamMutability::HotMutable,
         "Degrees either side of straight ahead that the sectors divide — the cone a gaze sweep "
         "actually reaches, wider than the sensor's own field.", ParamValue{40.0}, ParamValue{5.0}, ParamValue{180.0}},
        {"max_range", ParamMutability::HotMutable,
         "Beyond this the cloud is the room's walls rather than its contents (metres).",
         ParamValue{2.5}, ParamValue{0.2}, ParamValue{10.0}},
        {"view_half_fov", ParamMutability::HotMutable,
         "Degrees either side of straight ahead that view() divides into kSectors: the whole reach of a stop's "
         "gaze (a +-0.7 rad sweep plus half the sensor's 45 deg field).", ParamValue{64.0}, ParamValue{5.0}, ParamValue{180.0}},
        {"view_range", ParamMutability::HotMutable,
         "The range view() divides by (metres): 4 m, the scale of the frame's own nearest-hit-per-column, so a place "
         "map fed the cloud sees values on the scale it saw before.", ParamValue{4.0}, ParamValue{0.2}, ParamValue{10.0}},
        {"max_voxels", ParamMutability::HotMutable,
         "Hard cap on one cloud, so a runaway cannot eat memory.", ParamValue{int64_t{400000}}, ParamValue{int64_t{100}}, ParamValue{int64_t{5000000}}},
        {"move_ticks", ParamMutability::HotMutable,
         "Consecutive NOT-still ticks before an open cloud is filed.  Hysteresis, and it is "
         "load-bearing: a gaze babble jogs the trunk's gyro past any instantaneous stillness test "
         "every few seconds, and closing on the first such tick chopped one stop's sweep into three "
         "fragments of 50, 70 and 943 voxels (measured 2026-09-13, before this existed).  A cloud "
         "should end when the body LEAVES, not when it twitches.",
         ParamValue{int64_t{25}}, ParamValue{int64_t{1}}, ParamValue{int64_t{2000}}},
        {"still_ticks", ParamMutability::HotMutable,
         "Consecutive still ticks before a cloud opens.  The cloud's frame is only fixed while the "
         "body is, so this is the guard on the de-rotation's premise, not a schedule.",
         ParamValue{int64_t{25}}, ParamValue{int64_t{1}}, ParamValue{int64_t{2000}}},
        {"cache_size", ParamMutability::HotMutable,
         "How many filed clouds to keep, evicting least-recently-filed.  0 = no cache.",
         ParamValue{int64_t{8}}, ParamValue{int64_t{0}}, ParamValue{int64_t{256}}},
        {"things_topic", ParamMutability::ConstructionOnly,
         "Optional ProprioToken of kThing values describing the ATTENDED THING -- the nearest cluster of the "
         "open cloud whose stack tops out under small_top (the stack rule, design doc §17.31, run in the "
         "module): [top / break_hi, footprint / small_ext, aspect, columns / 25, hits per column / 20, "
         "chain / 5, range / max_range, lowest height / break_hi].  World-sized and without a bearing, so an "
         "EPM on it earns a vocabulary of THINGS rather than of poses (`microduck_things_phase.md` T1).  "
         "Published only while a thing is attended.  Empty = not computed.",
         ParamValue{std::string("")}},
        {"thing_bearing_topic", ParamMutability::ConstructionOnly,
         "Optional ProprioToken [vx = +right, vy = +forward, proximity] to the attended thing in the BODY "
         "frame this tick (the cloud's frame turned back by the yaw drift since the anchor) -- the shape "
         "VisualBearing emits, so VisualHomingNav consumes it unchanged.  proximity = 1 - range / "
         "things_range; all 0 when nothing small is in reach or no cloud is open.  Empty = not published.",
         ParamValue{std::string("")}},
        {"small_top", ParamMutability::HotMutable,
         "A cluster whose stack chain tops out below this (metres) is a small thing; one that keeps rising is "
         "an obstacle.  0.16 measured at precision 0.53 out of sample (0.92 with the sweep) at full recall.",
         ParamValue{0.16}, ParamValue{0.02}, ParamValue{1.0}},
        {"small_ext", ParamMutability::HotMutable,
         "...and whose footprint spans at most this (metres).", ParamValue{0.20}, ParamValue{0.04}, ParamValue{2.0}},
        {"small_ext_min", ParamMutability::HotMutable,
         "...and at least this (metres; 0 = no floor).  A one-voxel footprint (4 cm) is a fragment, not a thing: "
         "on R57 (n = 6) the attended cluster was a real object on 73 % of ticks with no floor, 86 % at two "
         "columns (0.08) and 93 % at three (0.12), the losses being wall bases and chair legs seen as single "
         "columns.  A real ball is one column early in a sweep too, so the floor delays attention until the "
         "thing is sampled, which is the point.", ParamValue{0.0}, ParamValue{0.0}, ParamValue{1.0}},
        {"things_shape", ParamMutability::ConstructionOnly,
         "Publish the SHAPE-ONLY descriptor (kThingShape = 5 dims: top, footprint, aspect, lowest height, chain) "
         "instead of the full one (kThing = 8, adding columns, hits per column and range).  See the header: "
         "with the sampling dims in, the vocabulary followed the sweep's fill-in state rather than the thing.",
         ParamValue{false}},
        {"gap_min", ParamMutability::HotMutable,
         "The stack chain's gap floor (metres): two heights further apart than max(gap_min, gap_k x range) are "
         "not one stack.", ParamValue{0.10}, ParamValue{0.0}, ParamValue{1.0}},
        {"gap_k", ParamMutability::HotMutable,
         "The gap's growth per metre of range: the sensor's rows are 5.6 deg apart, so the vertical spacing of "
         "returns up a face grows with distance.  0 = a fixed gap.", ParamValue{0.12}, ParamValue{0.0}, ParamValue{1.0}},
        {"things_range", ParamMutability::HotMutable,
         "A small thing is attended only within this range (metres).  0 = max_range.",
         ParamValue{0.0}, ParamValue{0.0}, ParamValue{10.0}},
        {"walk_cloud", ParamMutability::HotMutable,
         "Also accumulate while the body MOVES: casts translated by the odometry's displacement from the anchor, "
         "the cloud filed and reopened every walk_reset_m of travel, never cached as a place.  The things "
         "reduction runs on it, so a thing is attended on the walk (2026-09-19).  Off = byte-identical.",
         ParamValue{false}},
        {"walk_reset_m", ParamMutability::HotMutable,
         "Metres of travel after which a walking cloud is filed and a fresh one anchored (the odometry drifts "
         "4-6 % of distance: one voxel per metre).", ParamValue{1.0}, ParamValue{0.1}, ParamValue{10.0}},
        {"things_every", ParamMutability::HotMutable,
         "Recompute the clusters every this-many ticks while the cloud is open (the sensor casts every 4).",
         ParamValue{int64_t{4}}, ParamValue{int64_t{1}}, ParamValue{int64_t{1000}}},
        {"new_window_ticks", ParamMutability::HotMutable,
         "The window new_fraction asks about: of the voxels touched in the last this-many ticks, "
         "how many were first seen inside it.  A one-second window detected a rolling ball 45 % of "
         "the time at a matched 5 % false-positive rate offline — measured with the wrong-signed "
         "de-rotation, so re-measure before leaning on it.",
         ParamValue{int64_t{50}}, ParamValue{int64_t{2}}, ParamValue{int64_t{2000}}},
    };
}

ParamMap CloudMap::current_params() const {
    ParamMap m;
    m["input_topic"] = ParamValue{input_topic_};
    m["place_topic"] = ParamValue{place_topic_};
    m["output_topic"] = ParamValue{output_topic_};
    m["change_topic"] = ParamValue{change_topic_};
    m["voxel_m"] = voxel_m_;
    m["break_lo"] = break_lo_;
    m["break_hi"] = break_hi_;
    m["half_fov"] = half_fov_;
    m["max_range"] = max_range_;
    m["view_half_fov"] = view_half_fov_;
    m["view_range"] = view_range_;
    m["max_voxels"] = int64_t{max_voxels_};
    m["still_ticks"] = int64_t{still_ticks_};
    m["move_ticks"] = int64_t{move_ticks_};
    m["cache_size"] = int64_t{cache_size_};
    m["new_window_ticks"] = int64_t{new_window_};
    m["things_topic"] = ParamValue{things_topic_};
    m["thing_bearing_topic"] = ParamValue{thing_bearing_topic_};
    m["small_top"] = small_top_;
    m["small_ext"] = small_ext_;
    m["small_ext_min"] = small_ext_min_;
    m["things_shape"] = things_shape_;
    m["gap_min"] = gap_min_;
    m["gap_k"] = gap_k_;
    m["things_range"] = things_range_;
    m["things_every"] = int64_t{things_every_};
    m["walk_cloud"] = walk_cloud_;
    m["walk_reset_m"] = walk_reset_m_;
    return m;
}

void CloudMap::on_param_change(std::string_view key, ParamValue const& value) {
    const std::string k(key);
    ParamMap one{{k, value}};
    if      (k == "voxel_m")    voxel_m_ = get_d(one, "voxel_m", voxel_m_);
    else if (k == "break_lo")   break_lo_ = get_d(one, "break_lo", break_lo_);
    else if (k == "break_hi")   break_hi_ = get_d(one, "break_hi", break_hi_);
    else if (k == "half_fov")   half_fov_ = get_d(one, "half_fov", half_fov_);
    else if (k == "max_range")  max_range_ = get_d(one, "max_range", max_range_);
    else if (k == "view_half_fov") view_half_fov_ = get_d(one, "view_half_fov", view_half_fov_);
    else if (k == "view_range") view_range_ = get_d(one, "view_range", view_range_);
    else if (k == "max_voxels") max_voxels_ = int(get_d(one, "max_voxels", max_voxels_));
    else if (k == "still_ticks") still_ticks_ = int(get_d(one, "still_ticks", still_ticks_));
    else if (k == "move_ticks") move_ticks_ = int(get_d(one, "move_ticks", move_ticks_));
    else if (k == "cache_size") cache_size_ = int(get_d(one, "cache_size", cache_size_));
    else if (k == "new_window_ticks") new_window_ = int(get_d(one, "new_window_ticks", new_window_));
    else if (k == "small_top")  small_top_ = get_d(one, "small_top", small_top_);
    else if (k == "small_ext")  small_ext_ = get_d(one, "small_ext", small_ext_);
    else if (k == "small_ext_min") small_ext_min_ = get_d(one, "small_ext_min", small_ext_min_);
    else if (k == "gap_min")    gap_min_ = get_d(one, "gap_min", gap_min_);
    else if (k == "gap_k")      gap_k_ = get_d(one, "gap_k", gap_k_);
    else if (k == "things_range") things_range_ = get_d(one, "things_range", things_range_);
    else if (k == "things_every") things_every_ = std::max(1, int(get_d(one, "things_every", things_every_)));
    else if (k == "walk_cloud")  walk_cloud_ = get_d(one, "walk_cloud", 0.0) > 0.5;
    else if (k == "walk_reset_m") walk_reset_m_ = get_d(one, "walk_reset_m", walk_reset_m_);
}

void CloudMap::on_setup(Bus* bus, ParamMap const& params) {
    bus_ = bus;
    input_topic_  = get_s(params, "input_topic");
    place_topic_  = get_s(params, "place_topic");
    output_topic_ = get_s(params, "output_topic");
    change_topic_ = get_s(params, "change_topic");
    voxel_m_    = get_d(params, "voxel_m", voxel_m_);
    break_lo_   = get_d(params, "break_lo", break_lo_);
    break_hi_   = get_d(params, "break_hi", break_hi_);
    half_fov_   = get_d(params, "half_fov", half_fov_);
    max_range_  = get_d(params, "max_range", max_range_);
    view_half_fov_ = get_d(params, "view_half_fov", view_half_fov_);
    view_range_ = get_d(params, "view_range", view_range_);
    max_voxels_ = int(get_d(params, "max_voxels", max_voxels_));
    still_ticks_ = int(get_d(params, "still_ticks", still_ticks_));
    move_ticks_ = int(get_d(params, "move_ticks", move_ticks_));
    cache_size_ = int(get_d(params, "cache_size", cache_size_));
    new_window_ = int(get_d(params, "new_window_ticks", new_window_));
    things_topic_ = get_s(params, "things_topic");
    thing_bearing_topic_ = get_s(params, "thing_bearing_topic");
    small_top_  = get_d(params, "small_top", small_top_);
    small_ext_  = get_d(params, "small_ext", small_ext_);
    small_ext_min_ = get_d(params, "small_ext_min", small_ext_min_);
    things_shape_ = get_d(params, "things_shape", 0.0) > 0.5;
    gap_min_    = get_d(params, "gap_min", gap_min_);
    gap_k_      = get_d(params, "gap_k", gap_k_);
    things_range_ = get_d(params, "things_range", things_range_);
    things_every_ = std::max(1, int(get_d(params, "things_every", things_every_)));
    things_on_ = !things_topic_.empty() || !thing_bearing_topic_.empty();
    walk_cloud_ = get_d(params, "walk_cloud", 0.0) > 0.5;
    walk_reset_m_ = get_d(params, "walk_reset_m", walk_reset_m_);
}

void CloudMap::open_cloud(double anchor_yaw, double ax, double ay, uint64_t tick) {
    vox_.clear();
    winner_hist_.clear();
    anchor_yaw_ = anchor_yaw;
    anchor_x_ = ax;
    anchor_y_ = ay;
    opened_tick_ = tick;
    points_ = 0;
    break_vox_ = 0;
    new_frac_ = 0.0;
    revisit_change_ = -1.0;
    open_ = true;
}

void CloudMap::add_cast(const Eigen::VectorXf& v, double yaw, double trunk_z, uint64_t tick) {
    if (int(vox_.size()) >= max_voxels_) return;
    double d = yaw - anchor_yaw_;
    while (d > kPi) d -= 2.0 * kPi;
    while (d < -kPi) d += 2.0 * kPi;
    // R(+d): the body turned +d, so the world it sees is turned -d; turning each cast back by +d puts it
    // on the anchor's frame.  (It was R(-d) until 2026-09-13, which doubled the smear — see the header.)
    const double c = std::cos(d), s = std::sin(d);
    // a walking cloud: the body has also MOVED since the anchor; the displacement, turned into the anchor's frame
    double tx = 0.0, ty = 0.0;
    if (walking_cloud_ && v.size() >= 5) {
        const double dx = double(v[3]) - anchor_x_, dy = double(v[4]) - anchor_y_;
        const double ca = std::cos(-anchor_yaw_), sa = std::sin(-anchor_yaw_);
        tx = ca * dx - sa * dy; ty = sa * dx + ca * dy;
    }
    for (int i = 0; i < kZones; ++i) {
        const int b = 5 + 3 * i;
        if (b + 2 >= int(v.size())) break;
        const double px = double(v[b]), py = double(v[b + 1]), hz = double(v[b + 2]);
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(hz)) continue;
        const double x = c * px - s * py + tx;
        const double y = s * px + c * py + ty;
        if (std::hypot(x, y) > max_range_) continue;
        ++points_;
        const int ix = int(std::floor(x / voxel_m_));
        const int iy = int(std::floor(y / voxel_m_));
        const int iz = int(std::floor(hz / voxel_m_));
        auto& vv = vox_[key_of(ix, iy, iz)];
        if (vv.hits == 0) {
            vv.first = tick;
            if (hz >= break_lo_ && hz < break_hi_) ++break_vox_;
        }
        ++vv.hits;
        vv.zsum += float(hz);
        vv.last = tick;
    }
    (void)trunk_z;   // the host already folded it into z: the input's z IS height above the floor
}

void CloudMap::file_cloud(uint64_t tick) {
    open_ = false;
    just_closed_ = true;
    ++filed_count_;
    // the place this cloud belongs to: the MODAL winner while it was open
    int key = -1, best = 0;
    for (auto const& [w, n] : winner_hist_) if (n > best) { best = n; key = w; }
    last_key_ = key;
    // the viewer's payload, and the anchor it was built on
    filed_vox_.clear();
    filed_vox_.reserve(vox_.size() * 5);
    for (auto const& [k, vv] : vox_) {
        int x, y, z; unkey(k, x, y, z);
        filed_vox_.push_back(x); filed_vox_.push_back(y); filed_vox_.push_back(z);
        filed_vox_.push_back(int32_t(vv.hits));
        filed_vox_.push_back(int32_t(std::lround(1000.0 * double(vv.zsum) / double(std::max<uint32_t>(1, vv.hits)))));
    }
    filed_anchor_yaw_ = anchor_yaw_;
    filed_things_ = things_on_ ? cluster_things() : std::vector<Thing>{};
    things_.clear();
    attended_ = -1;
    bearing_ = {0.0f, 0.0f, 0.0f};
    // THE REVISIT JUDGEMENT, made once, on a finished cloud, against the place it is filed under.
    // Computed mid-accumulation it keyed onto whichever node happened to be winning first, which is
    // a different place's cloud (measured 2026-09-13: 0.86 either way, i.e. two frames that never
    // lined up).  A finished cloud and its own key is the only honest comparison.
    revisit_change_ = -1.0;
    revisit_overlap_ = 0;
    revisit_dist_ = -1.0;
    if (cache_size_ > 0 && key >= 0 && !vox_.empty() && !walking_cloud_) {
        auto prev = cache_.find(key);
        if (prev != cache_.end()) {
            const auto& oc = prev->second;
            revisit_dist_ = std::hypot(anchor_x_ - oc.ax, anchor_y_ - oc.ay);
            const double c1 = std::cos(anchor_yaw_), s1 = std::sin(anchor_yaw_);
            const double c2 = std::cos(-oc.ayaw), s2 = std::sin(-oc.ayaw);
            int missing = 0, overlap = 0;
            for (auto const& [k, vv] : vox_) {
                (void)vv;
                int ix, iy, iz; unkey(k, ix, iy, iz);
                const double px = (double(ix) + 0.5) * voxel_m_, py = (double(iy) + 0.5) * voxel_m_;
                const double wx = c1 * px - s1 * py + anchor_x_;
                const double wy = s1 * px + c1 * py + anchor_y_;
                const double dx = wx - oc.ax, dy = wy - oc.ay;
                const double qx = c2 * dx - s2 * dy, qy = s2 * dx + c2 * dy;
                const int jx = int(std::floor(qx / voxel_m_)), jy = int(std::floor(qy / voxel_m_));
                ++overlap;
                if (!oc.vox.count(key_of(jx, jy, iz))) ++missing;
            }
            revisit_overlap_ = overlap;
            revisit_change_ = overlap ? double(missing) / double(overlap) : -1.0;
        }
    }
    if (cache_size_ > 0 && key >= 0 && !walking_cloud_) {
        auto it = cache_.find(key);
        if (it == cache_.end() && int(cache_.size()) >= cache_size_ && !lru_.empty()) {
            cache_.erase(lru_.front());                      // least recently filed goes
            lru_.pop_front();
        }
        Cached c; c.vox = vox_; c.ax = anchor_x_; c.ay = anchor_y_; c.ayaw = anchor_yaw_; c.filed = tick;
        cache_[key] = std::move(c);
        lru_.erase(std::remove(lru_.begin(), lru_.end(), key), lru_.end());
        lru_.push_back(key);
    }
    vox_.clear();
    winner_hist_.clear();
}

void CloudMap::tick(uint64_t tick_id) {
    just_closed_ = false;
    if (!bus_ || input_topic_.empty()) return;
    auto pt = std::dynamic_pointer_cast<const ProprioToken>(bus_->last_value(input_topic_));
    if (!pt || pt->values.size() < 3) return;
    if (pt->values.size() < 5) return;
    const bool still = pt->values[0] > 0.5f;
    const double yaw = double(pt->values[1]);
    const double trunk_z = double(pt->values[2]);
    const double ox = double(pt->values[3]), oy = double(pt->values[4]);

    if (still) { ++still_run_; move_run_ = 0; } else { ++move_run_; still_run_ = 0; }
    if (walk_cloud_) {
        // a stop's cloud opens on stillness and files when the body leaves, as before; between stops a WALKING
        // cloud is open, translated by the odometry, filed and reopened every walk_reset_m of travel or when
        // the body becomes still (so the stop's cloud starts clean).  A walking cloud is never cached.
        if (open_ && !walking_cloud_ && move_run_ >= move_ticks_) { file_cloud(tick_id); open_cloud(yaw, ox, oy, tick_id); walking_cloud_ = true; }
        else if (open_ && walking_cloud_ && still_run_ >= still_ticks_) { file_cloud(tick_id); open_cloud(yaw, ox, oy, tick_id); walking_cloud_ = false; }
        else if (open_ && walking_cloud_ && std::hypot(ox - anchor_x_, oy - anchor_y_) > walk_reset_m_) { file_cloud(tick_id); open_cloud(yaw, ox, oy, tick_id); walking_cloud_ = true; }
        else if (!open_) { open_cloud(yaw, ox, oy, tick_id); walking_cloud_ = still_run_ < still_ticks_; }
        add_cast(pt->values, yaw, trunk_z, tick_id);
    } else {
        if (!open_ && still_run_ >= still_ticks_) open_cloud(yaw, ox, oy, tick_id);
        if (open_ && move_run_ >= move_ticks_) { file_cloud(tick_id); publish_bearing(tick_id); return; }
        if (!open_) { publish_bearing(tick_id); return; }
        // a twitch does not end the cloud, but nor does it contribute: the de-rotation's premise is a
        // still trunk, so a moving tick is simply skipped and the sweep resumes when the body settles.
        if (still) add_cast(pt->values, yaw, trunk_z, tick_id);
    }

    // the place, for the cache key: count every tick's winner while the cloud is open
    if (!place_topic_.empty())
        if (auto rt = std::dynamic_pointer_cast<const RealityToken>(bus_->last_value(place_topic_)))
            if (rt->winner_id >= 0) ++winner_hist_[rt->winner_id];

    // change WITHIN the sweep: of the voxels touched lately, how many are new
    {
        const uint64_t lo = tick_id > uint64_t(new_window_) ? tick_id - uint64_t(new_window_) : 0;
        int touched = 0, fresh = 0;
        for (auto const& [k, vv] : vox_) {
            (void)k;
            if (vv.last < lo) continue;
            ++touched;
            if (vv.first >= lo) ++fresh;
        }
        new_frac_ = touched ? double(fresh) / double(touched) : 0.0;
    }
    // The revisit judgement is made at FILE time, on a finished cloud — see file_cloud.

    if (things_on_) {
        if (tick_id % uint64_t(things_every_) == 0) update_things(yaw);
        else if (attended_ >= 0) update_bearing(yaw);   // the body's yaw drifts between recomputes
        if (!things_topic_.empty() && attended_ >= 0) {
            auto out = std::make_shared<ProprioToken>();
            out->tick_id = tick_id;
            out->producer_id = std::string(id());
            out->sensor = "thing";
            const auto d = thing_descriptor(things_[size_t(attended_)]);
            out->values = Eigen::VectorXf::Map(d.data(), long(d.size()));
            bus_->publish(things_topic_, out);
        }
        publish_bearing(tick_id);
    }

    if (!output_topic_.empty()) {
        auto out = std::make_shared<ProprioToken>();
        out->tick_id = tick_id;
        out->producer_id = std::string(id());
        out->sensor = "cloud";
        const auto p = profile();
        out->values = Eigen::VectorXf::Map(p.data(), long(p.size()));
        bus_->publish(output_topic_, out);
    }
    if (!change_topic_.empty()) {
        auto out = std::make_shared<ProprioToken>();
        out->tick_id = tick_id;
        out->producer_id = std::string(id());
        out->sensor = "cloud_change";
        out->values = Eigen::VectorXf(2);
        out->values[0] = float(new_frac_);
        out->values[1] = float(revisit_change_);
        bus_->publish(change_topic_, out);
    }
}

// The stack rule (design doc §17.31), as cloud_objects.py scores it offline, on the live voxels.
std::vector<CloudMap::Thing> CloudMap::cluster_things() const {
    std::vector<Thing> out;
    if (vox_.empty()) return out;
    struct Col { std::vector<std::pair<double, uint32_t>> hs; bool seed = false; };
    std::unordered_map<int64_t, Col> cols;   // keyed by (ix, iy, 0)
    for (auto const& [k, vv] : vox_) {
        int ix, iy, iz; unkey(k, ix, iy, iz);
        const double h = double(vv.zsum) / double(std::max<uint32_t>(1, vv.hits));
        if (h < break_lo_) continue;
        auto& c = cols[key_of(ix, iy, 0)];
        c.hs.emplace_back(h, vv.hits);
        if (h < break_hi_) c.seed = true;
    }
    std::unordered_map<int64_t, bool> seen;
    for (auto const& [k0, c0] : cols) {
        if (!c0.seed || seen.count(k0)) continue;
        std::vector<int64_t> stack{k0}, comp;
        seen[k0] = true;
        while (!stack.empty()) {
            const int64_t a = stack.back(); stack.pop_back();
            comp.push_back(a);
            int ax, ay, az; unkey(a, ax, ay, az);
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy) {
                    if (!dx && !dy) continue;
                    const int64_t b = key_of(ax + dx, ay + dy, 0);
                    auto it = cols.find(b);
                    if (it == cols.end() || !it->second.seed || seen.count(b)) continue;
                    seen[b] = true;
                    stack.push_back(b);
                }
        }
        Thing t;
        double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, sx = 0.0, sy = 0.0;
        t.lo = 1e9;
        for (int64_t k : comp) {
            int x, y, z; unkey(k, x, y, z);
            const double px = (double(x) + 0.5) * voxel_m_, py = (double(y) + 0.5) * voxel_m_;
            xmin = std::min(xmin, px); xmax = std::max(xmax, px);
            ymin = std::min(ymin, py); ymax = std::max(ymax, py);
            sx += px; sy += py;
            for (auto const& [h, n] : cols[k].hs) {
                t.hits += double(n);
                if (h >= break_lo_ && h < break_hi_) t.lo = std::min(t.lo, h);
            }
        }
        t.ncols = int(comp.size());
        t.cx = sx / double(comp.size()); t.cy = sy / double(comp.size());
        t.rng = std::hypot(t.cx, t.cy);
        const double ex = xmax - xmin + voxel_m_, ey = ymax - ymin + voxel_m_;
        t.ext = std::max(ex, ey); t.ext_min = std::min(ex, ey);
        // the chain: every height over the DILATED footprint, climbed from lo while each step is within the gap
        const double gap = std::max(gap_min_, gap_k_ * t.rng);
        std::vector<double> heights;
        std::unordered_map<int64_t, bool> dil;
        for (int64_t k : comp) {
            int x, y, z; unkey(k, x, y, z);
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy) {
                    const int64_t b = key_of(x + dx, y + dy, 0);
                    if (dil.count(b)) continue;
                    dil[b] = true;
                    auto it = cols.find(b);
                    if (it == cols.end()) continue;
                    for (auto const& [h, n] : it->second.hs) { (void)n; heights.push_back(h); }
                }
        }
        std::sort(heights.begin(), heights.end());
        t.top = t.lo;
        for (double h : heights) {
            if (h < t.lo) continue;
            if (h > t.top + gap) break;
            t.top = std::max(t.top, h);
        }
        t.chain = int(std::lround((t.top - t.lo) / voxel_m_)) + 1;
        t.small = t.top < small_top_ && t.ext <= small_ext_ && t.ext >= small_ext_min_ - 1e-9;
        out.push_back(t);
    }
    std::sort(out.begin(), out.end(), [](const Thing& a, const Thing& b) { return a.rng < b.rng; });
    return out;
}

std::vector<float> CloudMap::thing_descriptor(const Thing& t) const {
    const auto u = [](double v) { return float(std::clamp(v, 0.0, 1.0)); };
    if (things_shape_)
        return {u(t.top / break_hi_), u(t.ext / small_ext_), u(t.ext > 1e-9 ? t.ext_min / t.ext : 0.0),
                u(t.lo / break_hi_), u(double(t.chain) / 5.0)};
    return {u(t.top / break_hi_), u(t.ext / small_ext_), u(t.ext > 1e-9 ? t.ext_min / t.ext : 0.0),
            u(double(t.ncols) / 25.0), u(t.hits / double(std::max(1, t.ncols)) / 20.0),
            u(double(t.chain) / 5.0), u(t.rng / max_range_), u(t.lo / break_hi_)};
}

void CloudMap::update_bearing(double yaw) {
    bearing_ = {0.0f, 0.0f, 0.0f};
    if (attended_ < 0 || attended_ >= int(things_.size())) return;
    const Thing& t = things_[size_t(attended_)];
    if (t.rng < 1e-6) return;
    // the cloud's frame is the anchor's; the body has since yawed by d, so the thing it sees is turned by -d
    double d = yaw - anchor_yaw_;
    while (d > kPi) d -= 2.0 * kPi;
    while (d < -kPi) d += 2.0 * kPi;
    const double c = std::cos(-d), s = std::sin(-d);
    const double bx = c * t.cx - s * t.cy, by = s * t.cx + c * t.cy;   // body frame: x forward, y left
    const double reach = things_range_ > 0.0 ? things_range_ : max_range_;
    bearing_[0] = float(-by / t.rng);                                   // +right
    bearing_[1] = float(bx / t.rng);                                    // +forward
    bearing_[2] = float(std::clamp(1.0 - t.rng / reach, 0.0, 1.0));
}

// Every tick the topic exists, open cloud or not: while the body walks the cloud is closed and nothing is
// attended, and a consumer must read that as proximity 0 rather than the last stop's stale bearing.
void CloudMap::publish_bearing(uint64_t tick_id) {
    if (thing_bearing_topic_.empty()) return;
    auto out = std::make_shared<ProprioToken>();
    out->tick_id = tick_id;
    out->producer_id = std::string(id());
    out->sensor = "thing_bearing";
    out->values = Eigen::VectorXf(3);
    out->values[0] = bearing_[0]; out->values[1] = bearing_[1]; out->values[2] = bearing_[2];
    bus_->publish(thing_bearing_topic_, out);
}

void CloudMap::update_things(double yaw) {
    things_ = cluster_things();
    attended_ = -1;
    const double reach = things_range_ > 0.0 ? things_range_ : max_range_;
    for (size_t i = 0; i < things_.size(); ++i)                        // sorted by range: the first small one is the nearest
        if (things_[i].small && things_[i].rng <= reach) { attended_ = int(i); break; }
    update_bearing(yaw);
}

std::vector<float> CloudMap::view() const {
    std::vector<float> out(size_t(kSectors), 1.0f);
    const double span = 2.0 * view_half_fov_;
    for (auto const& [k, vv] : vox_) {
        int ix, iy, iz; unkey(k, ix, iy, iz);
        const double h = double(vv.zsum) / double(std::max<uint32_t>(1, vv.hits));   // mean point height
        if (h < break_lo_) continue;
        const double x = (double(ix) + 0.5) * voxel_m_;
        const double y = (double(iy) + 0.5) * voxel_m_;
        const double r = std::hypot(x, y);
        if (r < 1e-6) continue;
        const double az = std::atan2(y, x) * 180.0 / kPi;
        if (std::fabs(az) > view_half_fov_) continue;
        const int sec = std::clamp(int((az + view_half_fov_) / span * kSectors), 0, kSectors - 1);
        out[size_t(sec)] = std::min(out[size_t(sec)], float(std::clamp(r / view_range_, 0.0, 1.0)));
    }
    return out;
}

std::vector<float> CloudMap::profile() const {
    std::vector<float> out(size_t(kProfile), 0.0f);
    if (vox_.empty()) return out;
    std::vector<double> near(size_t(kSectors), max_range_), nh(size_t(kSectors), 0.0);
    std::vector<double> zlo(size_t(kSectors), 1e9), zhi(size_t(kSectors), -1e9);
    std::vector<int> mass(size_t(kSectors), 0);
    int total_break = 0, sectors_hit = 0;
    double hsum = 0.0;
    const double span = 2.0 * half_fov_;
    for (auto const& [k, vv] : vox_) {
        int ix, iy, iz; unkey(k, ix, iy, iz);
        const double x = (double(ix) + 0.5) * voxel_m_;
        const double y = (double(iy) + 0.5) * voxel_m_;
        const double h = double(vv.zsum) / double(std::max<uint32_t>(1, vv.hits));   // mean point height, not the centre
        if (h < break_lo_ || h >= break_hi_) continue;
        const double r = std::hypot(x, y);
        if (r < 1e-6 || r > max_range_) continue;
        const double az = std::atan2(y, x) * 180.0 / kPi;
        if (std::fabs(az) > half_fov_) continue;
        const int sec = std::clamp(int((az + half_fov_) / span * kSectors), 0, kSectors - 1);
        ++total_break; ++mass[size_t(sec)]; hsum += h;
        zlo[size_t(sec)] = std::min(zlo[size_t(sec)], h);
        zhi[size_t(sec)] = std::max(zhi[size_t(sec)], h);
        if (r < near[size_t(sec)]) { near[size_t(sec)] = r; nh[size_t(sec)] = h; }
    }
    for (int s = 0; s < kSectors; ++s) {
        const bool hit = mass[size_t(s)] > 0;
        if (hit) ++sectors_hit;
        out[size_t(s)]                = float(hit ? near[size_t(s)] / max_range_ : 1.0);
        out[size_t(kSectors + s)]     = float(hit ? std::clamp(nh[size_t(s)] / break_hi_, 0.0, 1.0) : 0.0);
        out[size_t(2 * kSectors + s)] = float(hit ? std::clamp((zhi[size_t(s)] - zlo[size_t(s)]) / break_hi_, 0.0, 1.0) : 0.0);
        out[size_t(3 * kSectors + s)] = float(std::clamp(double(mass[size_t(s)]) / 60.0, 0.0, 1.0));
    }
    out[size_t(4 * kSectors + 0)] = float(std::clamp(double(total_break) / 400.0, 0.0, 1.0));
    out[size_t(4 * kSectors + 1)] = float(double(sectors_hit) / double(kSectors));
    out[size_t(4 * kSectors + 2)] = float(total_break ? std::clamp(hsum / total_break / break_hi_, 0.0, 1.0) : 0.0);
    out[size_t(4 * kSectors + 3)] = float(std::clamp(double(vox_.size()) / 4000.0, 0.0, 1.0));
    return out;
}

nlohmann::json CloudMap::diag_lite() const {
    return {
        {"open", open_}, {"voxels", int(vox_.size())}, {"break", break_vox_},
        {"newfrac", new_frac_}, {"revisit", revisit_change_},
        {"cached", int(cache_.size())}, {"filed", filed_count_}, {"place", last_key_},
        {"overlap", revisit_overlap_}, {"revisit_dist", revisit_dist_},
        {"things", int(things_.size())}, {"walking", walking_cloud_},
        {"small", int(std::count_if(things_.begin(), things_.end(), [](const Thing& t) { return t.small; }))},
        {"attended", attended_ >= 0 && attended_ < int(things_.size()) ? things_[size_t(attended_)].rng : -1.0},
    };
}

nlohmann::json CloudMap::diag_snapshot() const {
    nlohmann::json j = diag_lite();
    j["voxel_m"] = voxel_m_;
    j["anchor_yaw"] = open_ ? anchor_yaw_ : filed_anchor_yaw_;
    j["points"] = points_;
    // the live cloud, as [ix, iy, iz, hits] quadruples — what a voxel viewer draws
    nlohmann::json v = nlohmann::json::array();
    for (auto const& [k, vv] : vox_) {
        int x, y, z; unkey(k, x, y, z);
        v.push_back({x, y, z, int(vv.hits), int(std::lround(1000.0 * double(vv.zsum) / double(std::max<uint32_t>(1, vv.hits))))});
    }
    j["vox"] = std::move(v);
    nlohmann::json places = nlohmann::json::array();
    for (auto const& [key, c] : cache_)
        places.push_back(nlohmann::json{{"place", key}, {"voxels", int(c.vox.size())},
                                        {"filed", c.filed}, {"ax", c.ax}, {"ay", c.ay}, {"ayaw", c.ayaw}});
    j["cache"] = std::move(places);
    return j;
}

}  // namespace ogma
