// DistressAccumulator: the perch x stall accumulator behind reality.proprio.distress.
#include <gtest/gtest.h>
#include "ogma/body/Distress.hpp"

using ogma::body::DistressAccumulator;

TEST(Distress, NoPlantedFootMeansNoProgressSoAnInvertedFlailingBodyBecomesDistressed) {
    // Upside down (tilt pi) while the leg odometry reports a phantom 0.76 m/s walk: counted as progress the
    // stall score stays 0 and distress never rises; with zero planted feet the displacement is not counted.
    DistressAccumulator legacy, gated;
    double dl = 0.0, dg = 0.0;
    for (int64_t t = 0; t < 2000; ++t) {
        dl = legacy.step(0.0, 0.76, 0.0, 3.14, 0.02, t);
        dg = gated.step(0.0, 0.76, 0.0, 3.14, 0.02, t, /*n_planted=*/0);
    }
    EXPECT_LT(dl, 0.01) << "the phantom walk reads as progress";
    EXPECT_GT(dg, 0.9) << "no planted foot: stalled and tilted, so distressed";
}

TEST(Distress, UnknownPlantedCountIsTheOriginalArithmetic) {
    DistressAccumulator a, b;
    for (int64_t t = 0; t < 1500; ++t) {
        const double va = a.step(0.01, 0.02 * (t % 7), 0.1, 0.2 + 0.001 * (t % 13), 0.02, t);
        const double vb = b.step(0.01, 0.02 * (t % 7), 0.1, 0.2 + 0.001 * (t % 13), 0.02, t, -1);
        ASSERT_EQ(va, vb) << "tick " << t;
    }
}

TEST(Distress, PlantedFeetLeaveTheOdometryCounting) {
    DistressAccumulator a, b;
    for (int64_t t = 0; t < 1500; ++t) {
        const double va = a.step(0.0, 0.05, 0.0, 0.25, 0.02, t);
        const double vb = b.step(0.0, 0.05, 0.0, 0.25, 0.02, t, 3);
        ASSERT_EQ(va, vb);
    }
}
