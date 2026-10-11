#pragma once
// =============================================================================
// LegKinematics.hpp  --  GDScript binding for ogma::body::fk_leg
// =============================================================================
//
// Port doc Phase 4, Order step (a): _fk_leg moves out of picrawler_body.gd so the sim
// and ogma_host share one forward-kinematics implementation. The gate is that the sim
// stays BYTE-IDENTICAL across the swap.
//
// A thin shim, and it must stay one -- all arithmetic lives in
// cpp_core/include/ogma/body/LegKinematics.hpp, which is bit-verified against the
// GDScript original (cpp_core/tests/body/leg_kinematics_parity_check.cpp).
//
// ⚠ THE ANCHORS ARE A CACHE, AND A STALE CACHE HERE IS SILENT. They are rebuilt by
// _build_leg() on every body construction, including the live [B] morphology swap
// through _rebuild_body(). picrawler_body.gd:4670 already records what this class of
// bug looks like -- "the orbit camera silently stopped responding to input after the
// first live body swap" -- and FK is worse: a stale cache does not stop responding, it
// returns confident poses for geometry that no longer exists, and
// feet_y_gravity_cmd_imu (a promoted input) keeps publishing plausible numbers.
//
// So the anchors are versioned rather than trusted: set_leg() marks a leg present,
// clear() drops all four, and fk() on an unset leg pushes an error rather than
// returning a default-constructed pose that would look like a body at the origin.

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "ogma/body/LegKinematics.hpp"

namespace godot {

class LegKinematics : public RefCounted {
    GDCLASS(LegKinematics, RefCounted)

public:
    // Drop every anchor. Call before repopulating, so a rebuild that fails part-way
    // leaves legs UNSET (which fk() reports) rather than half-old and half-new.
    void clear();
    void set_leg(int i, Vector3 hip1_world, Vector3 hip2_world, Vector3 knee_world,
                 Vector3 coxa_rest_origin, Vector3 upper_rest_origin,
                 Vector3 lower_rest_origin, Vector3 hip2_axis, Vector3 knee_axis);
    // [coxa, upper, lower] Transform3D -- the same shape _fk_leg() returned.
    Array fk(int i, double t1, double t2, double t3, double suspend_lift_y) const;
    // Diagnostic: how many legs currently hold anchors. 4 means ready.
    int  legs_set() const;

protected:
    static void _bind_methods();

private:
    ogma::body::LegAnchors a_[4];
    bool have_[4] = { false, false, false, false };
};

}  // namespace godot
