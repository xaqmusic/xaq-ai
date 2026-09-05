#include "DuckBody.hpp"

#include <cmath>
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
}

DuckBody::~DuckBody() {
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
        for (mjtSize i = 7; i < m_->nq; ++i) d_->qpos[i] += n(rng);
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

    for (int s = 0; s < substeps_; ++s) mj_step(m_, d_);
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

bool DuckBody::touching_wall() const {
    for (int i = 0; i < d_->ncon; ++i) {
        for (int g : {d_->contact[i].geom1, d_->contact[i].geom2}) {
            const char* name = mj_id2name(m_, mjOBJ_GEOM, g);
            if (name && std::string(name).rfind("wall", 0) == 0) return true;
        }
    }
    return false;
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
