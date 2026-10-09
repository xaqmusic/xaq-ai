#include "DuckBody.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <string>

namespace mjhost {

const char* const kPolicyJoints[kNumPolicyJoints] = {
    "left_hip_yaw",  "left_hip_roll",  "left_hip_pitch",  "left_knee",  "left_ankle",
    "neck_pitch",    "head_pitch",     "head_yaw",        "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
};

// STAND2: the trunk sits ~5 mm forward of the older pose so the centre of mass is
// over the ankle axis. Matches HOME_FRAME in microduck_rl and DEFAULT_POSITION in
// duck-control, mouth excluded.
const std::array<double, kNumPolicyJoints> kHomePose = {
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
};

namespace {

int require(const mjModel* m, mjtObj type, const char* name, const char* what) {
    const int id = mj_name2id(m, type, name);
    if (id < 0) throw std::runtime_error(std::string("model has no ") + what + " named '" + name + "'");
    return id;
}

// Rotate a world vector into the frame described by a wxyz quaternion — the inverse
// of applying the quaternion. Written out rather than taken from MuJoCo so it reads
// the same as microduck_rl's `quat_rotate_inverse`, which is what the policies were
// trained against.
std::array<double, 3> quat_rotate_inverse(const double* q, const std::array<double, 3>& v) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const std::array<double, 3> u{x, y, z};
    const double dot = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    const std::array<double, 3> cross{u[1] * v[2] - u[2] * v[1],
                                      u[2] * v[0] - u[0] * v[2],
                                      u[0] * v[1] - u[1] * v[0]};
    std::array<double, 3> out{};
    for (int i = 0; i < 3; ++i) out[i] = v[i] * (2.0 * w * w - 1.0) - cross[i] * (2.0 * w) + u[i] * (2.0 * dot);
    return out;
}

}  // namespace

DuckBody::DuckBody(const std::string& scene_path) {
    char err[1024] = {0};
    m_ = mj_loadXML(scene_path.c_str(), nullptr, err, sizeof(err));
    if (!m_) throw std::runtime_error("loading " + scene_path + ": " + err);
    d_ = mj_makeData(m_);

    // G3 — the brain tick has to be a whole number of physics steps, or the tick
    // length wobbles between two substep counts and every learning rate in the
    // graph is quoted against a moving unit.
    const double exact = (1.0 / kBrainHz) / m_->opt.timestep;
    substeps_ = int(std::lround(exact));
    if (std::fabs(exact - substeps_) > 1e-9) {
        mj_deleteData(d_);
        mj_deleteModel(m_);
        throw std::runtime_error("brain tick is not a whole number of physics steps");
    }

    // G4 — every index this host will use, resolved by name, once.
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        const int jid = require(m_, mjOBJ_JOINT, kPolicyJoints[i], "joint");
        qpos_adr_[i] = m_->jnt_qposadr[jid];
        qvel_adr_[i] = m_->jnt_dofadr[jid];
        actuator_[i] = require(m_, mjOBJ_ACTUATOR, kPolicyJoints[i], "actuator");
    }
    trunk_body_ = require(m_, mjOBJ_BODY, "trunk_base", "body");
    quat_adr_ = m_->sensor_adr[require(m_, mjOBJ_SENSOR, "orientation", "sensor")];
    gyro_adr_ = m_->sensor_adr[require(m_, mjOBJ_SENSOR, "angular-velocity", "sensor")];
    accel_adr_ = m_->sensor_adr[require(m_, mjOBJ_SENSOR, "imu_accel", "sensor")];
    head_body_ = m_->site_bodyid[require(m_, mjOBJ_SITE, "head_imu", "site")];
    neck_root_ = require(m_, mjOBJ_BODY, "neck", "body");

    // The playroom adds free bodies (balls, blocks) and a clock hinge after the robot in
    // qpos. Reset noise must touch the robot's joints only: a normal deviate on a ball's
    // quaternion is not "a slightly wrong pose". A qpos index is the robot's when the joint
    // owning it sits in the trunk's kinematic tree (body_rootid).
    const int robot_root = m_->body_rootid[trunk_body_];
    qpos_is_robot_.assign(m_->nq, 0);
    for (int j = 0; j < m_->njnt; ++j) {
        if (m_->body_rootid[m_->jnt_bodyid[j]] != robot_root) continue;
        const int adr = m_->jnt_qposadr[j];
        const int n = (m_->jnt_type[j] == mjJNT_FREE) ? 7 : (m_->jnt_type[j] == mjJNT_BALL) ? 4 : 1;
        for (int k = 0; k < n; ++k) qpos_is_robot_[adr + k] = 1;
    }
    for (int b = 0; b < m_->nbody; ++b) {
        const char* name = mj_id2name(m_, mjOBJ_BODY, b);
        if (name && std::string(name).rfind("obj_", 0) == 0) ++n_objects_;
    }
    {
        const int sid = mj_name2id(m_, mjOBJ_SENSOR, "head_gyro");
        head_gyro_adr_ = (sid >= 0) ? m_->sensor_adr[sid] : -1;
    }
    if (train_contacts) {
        // microduck_rl's FULL_COLLISION, the contacts the walking policy trained with: the soles frictional (condim 3, priority
        // 1, friction 1.0), every other robot collision geom frictionless (condim 1)
        const int robot_root2 = m_->body_rootid[trunk_body_];
        for (int g = 0; g < m_->ngeom; ++g) {
            if (!(m_->geom_contype[g] || m_->geom_conaffinity[g])) continue;
            if (m_->body_rootid[m_->geom_bodyid[g]] != robot_root2) continue;
            const char* n = mj_id2name(m_, mjOBJ_GEOM, g);
            const bool foot = n && (std::string(n) == "left_foot_collision" || std::string(n) == "right_foot_collision");
            if (foot) { m_->geom_condim[g] = 3; m_->geom_priority[g] = 1; m_->geom_friction[3 * g] = 1.0; }
            else m_->geom_condim[g] = 1;
        }
    }
    if (bam.on) bam_setup();
}
bool DuckBody::train_contacts = false;

// ---- BAM ---------------------------------------------------------------------------------------------------------------
DuckBody::BamConfig DuckBody::bam;

namespace {
// Rhoban bam, params/xl330/m6.json (branch mjlab_frictionloss, the one microduck_rl pins), and the XL330 actuator constants
// (bam/dynamixel/actuator.py)
constexpr double kBamKt = 0.36601349688984386, kBamR = 2.8113923539223227, kBamArmature = 0.0018077432831600838;
constexpr double kBamFrictionBase = 0.004771183165566, kBamFrictionStribeck = 0.004676345799486616;
constexpr double kBamLfMotor = 0.2667860954283698, kBamLfExternal = 8.515871897059342e-06;
constexpr double kBamLfMotorStribeck = 1.0722918395099123e-05, kBamLfExternalStribeck = 0.08077928978935671;
constexpr double kBamLfMotorQuad = 0.009972471242139415, kBamLfExternalQuad = 0.004902565732332559;
constexpr double kBamDthetaStribeck = 2.890372094130307, kBamAlpha = 8.683259907618984, kBamViscous = 0.005359668274599504;
constexpr double kBamErrorGain = (4096.0 / (2.0 * 3.14159265358979323846)) / (256.0 * 885.0);
constexpr double kBamMaxPwm = 1.0, kBamMaxCurrent = 1.75;
}  // namespace

void DuckBody::bam_setup() {
    // bam.mjlab.BamActuator.edit_spec: position actuators -> motors (torque in ctrl), force-limited at vin_max kt / R; the
    // joint's armature from BAM, its damping and frictionloss zeroed (rewritten every step), its friction constraint stiffened
    const double force_limit = bam.vin_max * kBamKt / kBamR;
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        const int a = actuator_[i];
        m_->actuator_gaintype[a] = mjGAIN_FIXED; m_->actuator_biastype[a] = mjBIAS_NONE;
        for (int k = 0; k < mjNGAIN; ++k) m_->actuator_gainprm[a * mjNGAIN + k] = 0.0;
        for (int k = 0; k < mjNBIAS; ++k) m_->actuator_biasprm[a * mjNBIAS + k] = 0.0;
        m_->actuator_gainprm[a * mjNGAIN] = 1.0;
        m_->actuator_forcelimited[a] = 1;
        m_->actuator_forcerange[2 * a] = -force_limit; m_->actuator_forcerange[2 * a + 1] = force_limit;
        m_->actuator_ctrllimited[a] = 0;
        for (int k = 0; k < 6; ++k) m_->actuator_gear[a * 6 + k] = (k == 0) ? 1.0 : 0.0;
        const int dof = qvel_adr_[i];
        m_->dof_armature[dof] = kBamArmature; m_->dof_damping[dof] = 0.0; m_->dof_frictionloss[dof] = 0.0;
        m_->dof_solref[dof * mjNREF] = -5.0e4; m_->dof_solref[dof * mjNREF + 1] = -2.0e2;
        const double imp[5] = {0.99, 0.9999, 0.001, 0.5, 2.0};
        for (int k = 0; k < 5 && k < mjNIMP; ++k) m_->dof_solimp[dof * mjNIMP + k] = imp[k];
    }
    mj_setConst(m_, d_);
    if (!bam.dump.empty()) bam_dump_ = std::fopen(bam.dump.c_str(), "w");
}

void DuckBody::bam_update(const std::array<double, kNumPolicyJoints>& target) {
    // bam.mjlab.BamActuator.compute, one physics step, every actuated joint (no voltage drop, no gain randomisation)
    std::vector<double> fric(size_t(m_->nv), 0.0);        // the dof-friction constraint force of the previous solve
    for (int r = 0; r < d_->nefc; ++r)
        if (d_->efc_type[r] == mjCNSTR_FRICTION_DOF) fric[size_t(d_->efc_id[r])] += d_->efc_force[r];
    for (int i = 0; i < kNumPolicyJoints; ++i) {
        const int dof = qvel_adr_[i];
        const double q = d_->qpos[qpos_adr_[i]], dq = d_->qvel[dof];
        const double prev = d_->qfrc_actuator[dof];
        // 1-2. the firmware P loop (duty cycle), its current limiter, the PWM clamp; the DC motor with back-EMF
        double duty = (target[size_t(i)] - q) * bam.kp_fw * kBamErrorGain;
        const double span = kBamR * kBamMaxCurrent / bam.vin, centre = kBamKt * dq / bam.vin;
        duty = std::clamp(duty, centre - span, centre + span);
        duty = std::clamp(duty, -kBamMaxPwm, kBamMaxPwm);
        const double volts = bam.vin * duty;
        const double tau = kBamKt * volts / kBamR - kBamKt * kBamKt * dq / kBamR;
        d_->ctrl[actuator_[i]] = tau; bam_tau_[size_t(i)] = tau;
        // 3. the external load on the gearbox: bias + constraints, less our own friction constraint
        const double ext = -d_->qfrc_bias[dof] + d_->qfrc_constraint[dof] - fric[size_t(dof)];
        // 4-5. the m6 friction budget -> frictionloss; the viscous term -> damping
        const double strib = std::exp(-std::pow(std::fabs(dq) / kBamDthetaStribeck, kBamAlpha));
        double fl = kBamFrictionBase + strib * kBamFrictionStribeck;
        fl += std::fabs(ext * kBamLfExternal - prev * kBamLfMotor);
        fl += strib * std::fabs(ext * kBamLfExternalStribeck - prev * kBamLfMotorStribeck);
        const bool drive = std::fabs(prev) > std::fabs(ext);
        fl += strib * (drive ? kBamLfExternalQuad * ext * ext : kBamLfMotorQuad * prev * prev);
        m_->dof_frictionloss[dof] = fl; m_->dof_damping[dof] = kBamViscous;
    }
}

std::array<double, 3> DuckBody::head_gyro() const {
    if (head_gyro_adr_ < 0) return {0.0, 0.0, 0.0};
    const double* w = &d_->sensordata[head_gyro_adr_];
    return {w[0], w[1], w[2]};
}

DuckBody::~DuckBody() {
    if (bam_dump_) std::fclose(bam_dump_);
    if (d_) mj_deleteData(d_);
    if (m_) mj_deleteModel(m_);
}

void DuckBody::reset(const std::string& keyframe, double joint_noise, uint64_t seed) {
    const int key = require(m_, mjOBJ_KEY, keyframe.c_str(), "keyframe");
    mj_resetDataKeyframe(m_, d_, key);

    if (joint_noise > 0.0) {
        // The first seven qpos entries are the trunk's free joint (3 position,
        // 4 quaternion). Perturbing those would drop or rotate the whole robot,
        // which is a different experiment from starting in a slightly wrong pose.
        std::mt19937_64 rng(seed);
        std::normal_distribution<double> n(0.0, joint_noise);
        for (mjtSize i = 7; i < m_->nq; ++i)
            if (qpos_is_robot_[i]) d_->qpos[i] += n(rng);      // the same draws as before on a robot-only scene
    }
    mj_forward(m_, d_);
}

void DuckBody::push(const std::array<double, 3>& force_newtons, int ticks) {
    push_ = force_newtons;
    push_ticks_ = ticks;
}

void DuckBody::step(const std::array<double, kNumPolicyJoints>& ctrl) {
    for (int i = 0; i < kNumPolicyJoints; ++i) d_->ctrl[actuator_[i]] = ctrl[i];

    // xfrc_applied is a persistent field, so it is written every tick and cleared
    // when the window closes. Leaving a stale force on the trunk would look like a
    // controller that had developed a lean.
    const bool pushing = push_ticks_ > 0;
    for (int i = 0; i < 3; ++i) d_->xfrc_applied[6 * trunk_body_ + i] = pushing ? push_[i] : 0.0;
    if (pushing) --push_ticks_;

    if (!bam.on) { for (int s = 0; s < substeps_; ++s) mj_step(m_, d_); return; }
    if (bam_dump_) {
        // the validation's input: this tick's targets, then the robot's state before the physics runs
        std::fprintf(bam_dump_, "%ld", bam_tick_++);
        for (int i = 0; i < kNumPolicyJoints; ++i) std::fprintf(bam_dump_, " %.17g", ctrl[size_t(i)]);
        for (int k = 0; k < m_->nq; ++k) std::fprintf(bam_dump_, " %.17g", d_->qpos[k]);
        for (int k = 0; k < m_->nv; ++k) std::fprintf(bam_dump_, " %.17g", d_->qvel[k]);
        std::fprintf(bam_dump_, "\n");
    }
    const int lag = int(std::lround(bam.lag_s / m_->opt.timestep));
    for (int s = 0; s < substeps_; ++s) {
        bam_queue_.push_back(ctrl);
        const auto& tgt = bam_queue_.size() > size_t(lag) ? bam_queue_[bam_queue_.size() - 1 - size_t(lag)] : bam_queue_.front();
        bam_update(tgt);
        if (bam_queue_.size() > size_t(lag) + 1) bam_queue_.erase(bam_queue_.begin());
        if (bam_dump_ && s == 0) {   // the first substep's torques and friction budget, for the validation
            std::fprintf(bam_dump_, "T");
            for (int i = 0; i < kNumPolicyJoints; ++i) std::fprintf(bam_dump_, " %.17g", bam_tau_[size_t(i)]);
            for (int i = 0; i < kNumPolicyJoints; ++i) std::fprintf(bam_dump_, " %.17g", m_->dof_frictionloss[qvel_adr_[i]]);
            std::fprintf(bam_dump_, "\n");
        }
        mj_step(m_, d_);
    }
}

std::array<double, kNumPolicyJoints> DuckBody::joint_positions() const {
    std::array<double, kNumPolicyJoints> q{};
    for (int i = 0; i < kNumPolicyJoints; ++i) q[i] = d_->qpos[qpos_adr_[i]];
    return q;
}

std::array<double, kNumPolicyJoints> DuckBody::joint_velocities() const {
    std::array<double, kNumPolicyJoints> v{};
    for (int i = 0; i < kNumPolicyJoints; ++i) v[i] = d_->qvel[qvel_adr_[i]];
    return v;
}

std::array<double, 4> DuckBody::imu_quat() const {
    const double* q = &d_->sensordata[quat_adr_];
    return {q[0], q[1], q[2], q[3]};
}

void DuckBody::move_geom(const char* name, const std::array<double, 3>& pos) {
    const int gid = mj_name2id(m_, mjOBJ_GEOM, name);
    if (gid < 0) throw std::runtime_error(std::string("no geom ") + name);
    for (int i = 0; i < 3; ++i) m_->geom_pos[3 * gid + i] = pos[i];
    mj_forward(m_, d_);
}

// Both instruments ask about contacts the ROBOT is in. A ball resting on the floor or a
// block leaning on a chair is a contact too, and counting those made `obj` true on every
// tick of the first playroom runs (2026-09-10) and would let a block against a chair leg
// count as a wall contact. In the arena every contact involves the robot, so this is
// byte-identical there.
bool DuckBody::robot_contact(int i, int& other_geom) const {
    const int robot_root = m_->body_rootid[trunk_body_];
    const int g1 = d_->contact[i].geom1, g2 = d_->contact[i].geom2;
    const bool r1 = m_->body_rootid[m_->geom_bodyid[g1]] == robot_root;
    const bool r2 = m_->body_rootid[m_->geom_bodyid[g2]] == robot_root;
    if (r1 == r2) return false;                       // self-contact, or none of the robot
    other_geom = r1 ? g2 : g1;
    return true;
}

bool DuckBody::touching_wall() const {
    // Any static world geom that is not the floor or a rug: the arena's walls (as before —
    // there the static geoms are exactly floor + wall_*), and the playroom's furniture.
    for (int i = 0; i < d_->ncon; ++i) {
        int g;
        if (!robot_contact(i, g)) continue;
        if (m_->body_weldid[m_->geom_bodyid[g]] != 0) continue;          // moving body: not a wall
        const char* name = mj_id2name(m_, mjOBJ_GEOM, g);
        const std::string n = name ? name : "";
        if (n.rfind("floor", 0) == 0 || n.rfind("rug", 0) == 0) continue;
        return true;
    }
    return false;
}

bool DuckBody::touching_object() const {
    for (int i = 0; i < d_->ncon; ++i) {
        int g;
        if (!robot_contact(i, g)) continue;
        const char* name = mj_id2name(m_, mjOBJ_BODY, m_->body_rootid[m_->geom_bodyid[g]]);
        if (name && std::string(name).rfind("obj_", 0) == 0) return true;
    }
    return false;
}

void DuckBody::move_body(const char* name, double x, double y) {
    const int bid = mj_name2id(m_, mjOBJ_BODY, name);
    if (bid >= 0) {
        if (m_->body_jntnum[bid] > 0 && m_->jnt_type[m_->body_jntadr[bid]] == mjJNT_FREE) {
            const int adr = m_->jnt_qposadr[m_->body_jntadr[bid]];
            const int dof = m_->jnt_dofadr[m_->body_jntadr[bid]];
            d_->qpos[adr] = x; d_->qpos[adr + 1] = y;                     // z and the quaternion kept
            for (int k = 0; k < 6; ++k) d_->qvel[dof + k] = 0.0;
        } else {
            m_->body_pos[3 * bid] = x; m_->body_pos[3 * bid + 1] = y;    // a static furniture body
        }
        mj_forward(m_, d_);
        return;
    }
    const int gid = mj_name2id(m_, mjOBJ_GEOM, name);
    if (gid < 0) throw std::runtime_error(std::string("no body or geom ") + name);
    move_geom(name, {x, y, m_->geom_pos[3 * gid + 2]});
}

void DuckBody::roll_body(const char* name, double x, double y, double vx, double vy) {
    const int bid = mj_name2id(m_, mjOBJ_BODY, name);
    if (bid < 0 || m_->body_jntnum[bid] == 0 || m_->jnt_type[m_->body_jntadr[bid]] != mjJNT_FREE)
        throw std::runtime_error(std::string("roll_body: no movable ") + name);
    const int adr = m_->jnt_qposadr[m_->body_jntadr[bid]];
    const int dof = m_->jnt_dofadr[m_->body_jntadr[bid]];
    d_->qpos[adr] = x; d_->qpos[adr + 1] = y;
    for (int k = 0; k < 6; ++k) d_->qvel[dof + k] = 0.0;
    d_->qvel[dof] = vx; d_->qvel[dof + 1] = vy;
    mj_forward(m_, d_);
}

void DuckBody::place_free_body(const char* name, double x, double y, double z, double yaw, double vx, double vy, double wz) {
    const int bid = mj_name2id(m_, mjOBJ_BODY, name);
    if (bid < 0 || m_->body_jntnum[bid] == 0 || m_->jnt_type[m_->body_jntadr[bid]] != mjJNT_FREE)
        throw std::runtime_error(std::string("place_free_body: no movable ") + name);
    const int adr = m_->jnt_qposadr[m_->body_jntadr[bid]];
    const int dof = m_->jnt_dofadr[m_->body_jntadr[bid]];
    d_->qpos[adr] = x; d_->qpos[adr + 1] = y; d_->qpos[adr + 2] = z;
    d_->qpos[adr + 3] = std::cos(0.5 * yaw); d_->qpos[adr + 4] = 0.0; d_->qpos[adr + 5] = 0.0; d_->qpos[adr + 6] = std::sin(0.5 * yaw);
    for (int k = 0; k < 6; ++k) d_->qvel[dof + k] = 0.0;
    d_->qvel[dof] = vx; d_->qvel[dof + 1] = vy; d_->qvel[dof + 5] = wz;
    mj_forward(m_, d_);
}

std::vector<double> DuckBody::numeric(const char* name) const {
    const int id = mj_name2id(m_, mjOBJ_NUMERIC, name);
    if (id < 0) return {};
    const int adr = m_->numeric_adr[id], n = m_->numeric_size[id];
    return std::vector<double>(m_->numeric_data + adr, m_->numeric_data + adr + n);
}

std::array<double, 2> DuckBody::body_xy(const char* name) const {
    const int bid = mj_name2id(m_, mjOBJ_BODY, name);
    if (bid < 0) throw std::runtime_error(std::string("body_xy: no body ") + name);
    return {d_->xpos[3 * bid], d_->xpos[3 * bid + 1]};
}

std::array<double, 3> DuckBody::com_over_feet() const {
    const int root = mj_name2id(m_, mjOBJ_BODY, "trunk_base");
    const int gl = mj_name2id(m_, mjOBJ_GEOM, "left_foot_collision"), gr = mj_name2id(m_, mjOBJ_GEOM, "right_foot_collision");
    if (root < 0 || gl < 0 || gr < 0) return {0.0, 0.0, 0.0};
    const double cx = d_->subtree_com[3 * root], cy = d_->subtree_com[3 * root + 1], cz = d_->subtree_com[3 * root + 2];
    const double fx = 0.5 * (d_->geom_xpos[3 * gl] + d_->geom_xpos[3 * gr]), fy = 0.5 * (d_->geom_xpos[3 * gl + 1] + d_->geom_xpos[3 * gr + 1]);
    const double fz = 0.5 * (d_->geom_xpos[3 * gl + 2] + d_->geom_xpos[3 * gr + 2]);
    const double yaw = trunk_yaw(), c = std::cos(yaw), s = std::sin(yaw);
    const double dx = cx - fx, dy = cy - fy;
    return {c * dx + s * dy, -s * dx + c * dy, cz - fz};
}

double DuckBody::trunk_yaw() const {
    const double* q = d_->qpos + 3;   // the free joint's quaternion w, x, y, z
    return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

void DuckBody::spin_joint(const char* name, double rad_per_s) {
    const int jid = mj_name2id(m_, mjOBJ_JOINT, name);
    if (jid < 0) throw std::runtime_error(std::string("no joint ") + name);
    d_->qvel[m_->jnt_dofadr[jid]] = rad_per_s;
}

void DuckBody::site_world(const char* site, std::array<double, 3>& pos, std::array<double, 9>& mat) const {
    const int sid = mj_name2id(m_, mjOBJ_SITE, site);
    if (sid < 0) throw std::runtime_error(std::string("no site ") + site);
    for (int i = 0; i < 3; ++i) pos[i] = d_->site_xpos[3 * sid + i];
    for (int i = 0; i < 9; ++i) mat[i] = d_->site_xmat[9 * sid + i];
}

void DuckBody::site_pose_trunk(const char* site, std::array<double, 3>& pos,
                               std::array<double, 4>& quat) const {
    const int sid = mj_name2id(m_, mjOBJ_SITE, site);
    if (sid < 0) throw std::runtime_error(std::string("no site ") + site);
    const double* ps = &d_->site_xpos[3 * sid];
    const double* Rs = &d_->site_xmat[9 * sid];
    const double* pt = &d_->xpos[3 * trunk_body_];
    const double* Rt = &d_->xmat[9 * trunk_body_];
    double rel[3] = {ps[0] - pt[0], ps[1] - pt[1], ps[2] - pt[2]};
    double prel[3];
    mju_mulMatTVec(prel, Rt, rel, 3, 3);                 // Rtᵀ (ps − pt)
    double Rrel[9];
    mju_mulMatTMat(Rrel, Rt, Rs, 3, 3, 3);               // Rtᵀ Rs
    double q[4];
    mju_mat2Quat(q, Rrel);
    pos = {prel[0], prel[1], prel[2]};
    quat = {q[0], q[1], q[2], q[3]};
}

std::array<double, 3> DuckBody::gravity() const {
    return quat_rotate_inverse(&d_->sensordata[quat_adr_], {0.0, 0.0, -1.0});
}

std::array<double, 3> DuckBody::accel() const {
    return {d_->sensordata[accel_adr_], d_->sensordata[accel_adr_ + 1],
            d_->sensordata[accel_adr_ + 2]};
}

// Projected gravity IN THE HEAD FRAME.  The hardware has exactly one IMU (the
// trunk imu_to_dxl board); the head-frame attitude it cannot sense directly is
// nevertheless a rigid-body identity: head_quat = trunk_quat ∘ FK(measured neck
// and head joints), which upstream's own kinematics crate computes (the ToF
// Reprojector runs this very reduction).  Reading the sim's head body xquat is a
// transparent shortcut for that composition — same rigid chain, same measured
// joints — not a new sensor.  The head_imu CAD frame this reads at is the ghost
// of the dropped v1-lineage head IMU; here it becomes a derived channel instead.
std::array<double, 3> DuckBody::head_gravity() const {
    return quat_rotate_inverse(&d_->xquat[4 * head_body_], {0.0, 0.0, -1.0});
}

// Head-subtree CoM offset in the TRUNK frame (x fore/aft, y lateral).  Pure FK
// of measured joint angles plus CAD-constant masses — computable on hardware
// with no IMU at all — read here from MuJoCo's subtree_com as the transparent
// shortcut.  This is the observation the single trunk IMU is structurally blind
// to: trunk level + head craned forward reads ZERO lean while 38 % of the mass
// is far displaced.
std::array<double, 2> DuckBody::head_com_trunk() const {
    const double* com = &d_->subtree_com[3 * neck_root_];
    const double* tp  = &d_->xpos[3 * trunk_body_];
    const std::array<double, 3> rel = {com[0] - tp[0], com[1] - tp[1], com[2] - tp[2]};
    const auto local = quat_rotate_inverse(&d_->xquat[4 * trunk_body_], rel);
    return {local[0], local[1]};
}

std::array<double, 3> DuckBody::gyro() const {
    return {d_->sensordata[gyro_adr_], d_->sensordata[gyro_adr_ + 1], d_->sensordata[gyro_adr_ + 2]};
}

double DuckBody::tilt_deg() const {
    // Row-major 3x3; element (2,2) is the world-z component of the body's own z axis.
    const double cos_tilt = d_->xmat[9 * trunk_body_ + 8];
    return std::acos(std::fmax(-1.0, std::fmin(1.0, cos_tilt))) * 180.0 / M_PI;
}

std::vector<double> DuckBody::qpos() const {
    return std::vector<double>(d_->qpos, d_->qpos + m_->nq);
}

std::vector<double> DuckBody::qvel() const {
    return std::vector<double>(d_->qvel, d_->qvel + m_->nv);
}

void DuckBody::set_full_state(const std::vector<double>& qpos, const std::vector<double>& qvel) {
    if (int(qpos.size()) != m_->nq || int(qvel.size()) != m_->nv)
        throw std::runtime_error("set_full_state: size mismatch (snapshot from a different model?)");
    std::copy(qpos.begin(), qpos.end(), d_->qpos);
    std::copy(qvel.begin(), qvel.end(), d_->qvel);
    mj_forward(m_, d_);
}

std::array<double, 3> DuckBody::trunk_position() const {
    return {d_->xpos[3 * trunk_body_], d_->xpos[3 * trunk_body_ + 1], d_->xpos[3 * trunk_body_ + 2]};
}

}  // namespace mjhost
