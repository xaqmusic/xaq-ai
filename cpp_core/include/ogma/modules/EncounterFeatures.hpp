// =============================================================================
// EncounterFeatures.hpp  --  one vector per touch: what the body learned in the half second after a toe met something
// =============================================================================
//
// Palpation stage 1 (robot resume, 2026-11-01/02).  Three bench passes measured what tells a floor, a box top, a
// box face, a pillow and a wall apart through this body's own senses, and it was never the sole's load alone:
//   - WHERE the toe was when it fired: its height over the support plane (a top), its place ahead of the body;
//   - HOW it arrived: a vertical descent meets a face edge-on and loads the servo (current) before the sole; a
//     horizontal advance meets it sole-first;
//   - WHAT THE BODY DID next: a surface that resists makes the servo work (current rises 0.045 A per commanded mm
//     on a box, 0.004 on a pillow), lifts the belly (0.65 vs 0.15 mm/mm) and leans it; a soft one yields.
// This module watches the streams TouchMemory watches plus the bus current, the belly ToF, the attitude and the
// forward range, keeps a short pre-buffer, and on each foot's contact onset opens a window; when it closes it
// publishes ONE ProprioToken on `output_topic` (default sense.touch_encounter -- a sense.* name on purpose: the
// reality.proprio.* prefix is subscribed wholesale by other modules):
//
//   [0] h_floor_m        toe height over the FLOOR at onset: the belly plane minus the ToF clearance when the ToF reads,
//                        else the planted feet's plane (TouchMemory's rule); the floor calibration applied
//   [1] fwd_m, [2] lat_m toe position in the body frame at onset
//   [3] load_peak        the foot's peak load over the window (fraction of body weight, foot_load)
//   [4] i_rise_pre_a     bus-current rise over the pre-buffer up to onset (a face met edge-on loads the servo first)
//   [5] i_rise_post_a    bus-current rise over the window (the press's effort)
//   [6] belly_rise_m     belly-ToF clearance rise over the window (the body lifted)
//   [7] lean_rise_rad    attitude change over the window (the body rolled)
//   [8] range_m          forward range at onset (-1 = no echo / none)
//   [9] vert_frac        the toe's approach in the pre-buffer: |dy| / (|dy| + |dxz|), 1 = straight down, 0 = sideways
//
// An EPM over this topic is the ENCOUNTER VOCABULARY (CLAUDE.md §0: no hand classes -- what a box or a wall is, the
// GNG earns), a BeliefFilter over that EPM carries the belief between sparse events, and nothing here decides what
// anything is.  Publishes only at window ends; absent from a config = byte-identical.
#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "ogma/Module.hpp"
#include "ogma/Topics.hpp"

namespace ogma {

class EncounterFeatures : public Module {
public:
    static constexpr int kDim = 10;
    EncounterFeatures();
    ~EncounterFeatures() override;
    std::string_view             type_name()      const override;
    std::vector<TopicSpec>       input_topics()   const override;
    std::vector<TopicSpec>       output_topics()  const override;
    ParamSchema                  params_schema()  const override;
    ParamMap                     current_params() const override;
    void on_setup(Bus* bus, ParamMap const& params) override;
    void tick(uint64_t tick_id) override;
    void on_param_change(std::string_view key, ParamValue const& value) override;
    nlohmann::json diag_snapshot() const override;

    int64_t events() const { return events_; }
    std::array<float, kDim> const& last() const { return last_; }
    int last_leg() const { return last_leg_; }

private:
    struct Sample { float cur; float belly; float lean; std::array<float, 8> xz; std::array<float, 4> y; };
    struct Window { bool open = false; uint64_t t_on = 0; int ticks = 0; float h = 0, fwd = 0, lat = 0, i_pre = 0, i0 = 0, belly0 = 0, lean0 = 0, range = -1, vert = 0;
                    float fwd_rel = 0, lat_rel = 0;    // the toe's position relative to this leg's running mean of touch positions
                    float load_peak = 0, i_max = 0, belly_max = 0, lean_max = 0; };
    void handle(std::string const& which, MessagePtr p);
    void publish(int leg, Window const& w, uint64_t tick_id);

    std::string contact_topic_ = "reality.proprio.foot_contact";
    std::string feet_xz_topic_ = "reality.proprio.feet_xz";
    std::string feet_y_topic_  = "reality.proprio.feet_y_gravity_cmd_imu";
    std::string load_topic_    = "reality.proprio.foot_load";
    std::string current_topic_ = "sense.servo_current";
    std::string belly_topic_   = "reality.proprio.belly_m";
    std::string upright_topic_ = "reality.proprio.upright";
    std::string range_topic_   = "sense.range";
    std::string output_topic_  = "sense.touch_encounter";
    int    pre_ticks_  = 15;      // 0.3 s at 50 Hz
    int    post_ticks_ = 25;      // 0.5 s
    double plane_tau_ticks_ = 50.0;
    double belly_plane_m_ = -0.029;   // the belly plane below the body origin: the ToF's floor reference
    double leg_y_offset_[4] = {0, 0, 0, 0};
    // Conditioning (§0 rule 2): a touch's position is published relative to the leg's own running mean of touch
    // positions, so the vocabulary is about what was touched rather than which leg touched it.  Default off.
    bool   position_relative_ = false;
    double position_tau_touches_ = 20.0;
    int    recent_max_ = 32;                 // the diag's ring of recent events: [tick, leg, f0..f9]

    // latest inputs
    std::array<float, 4> contact_{}, prev_contact_{}, feet_y_{}, load_{};
    std::array<float, 8> feet_xz_{};
    float cur_ = 0, belly_ = 0, upright_ = 1, range_ = -1; bool range_valid_ = false;
    bool have_contact_ = false, have_xz_ = false, have_y_ = false, have_cur_ = false, have_belly_ = false, have_up_ = false;
    double plane_y_ = 0; bool have_plane_ = false;
    std::deque<Sample> pre_;
    std::array<Window, 4> win_{};
    std::array<float, kDim> last_{}; int last_leg_ = -1; int64_t events_ = 0; uint64_t last_tick_ = 0;
    std::array<std::array<double, 2>, 4> pos_mean_{}; std::array<int, 4> pos_n_{};
    std::deque<std::array<float, kDim + 2>> recent_;
    Bus* bus_ = nullptr; std::vector<uint64_t> sub_ids_;
};

} // namespace ogma
