// ogma::body::DeadReckon -- the odom frame and the integrator, pinned.
#include "ogma/body/DeadReckon.hpp"
#include <gtest/gtest.h>
#include <cmath>
using ogma::body::DeadReckon;

TEST(DeadReckon, ForwardAtZeroHeadingMovesAlongXo) {
    DeadReckon d;
    for (int i = 0; i < 50; ++i) d.step(0.0, 0.2, 0.0, 0.02);   // 1 s at 0.2 m/s
    EXPECT_NEAR(d.x(), 0.2, 1e-12);
    EXPECT_NEAR(d.y(), 0.0, 1e-12);
    EXPECT_NEAR(d.dist(), 0.2, 1e-12);
}

TEST(DeadReckon, BodyPlusXAtZeroHeadingMovesAlongYo) {
    DeadReckon d;
    for (int i = 0; i < 50; ++i) d.step(0.1, 0.0, 0.0, 0.02);
    EXPECT_NEAR(d.x(), 0.0, 1e-12);
    EXPECT_NEAR(d.y(), 0.1, 1e-12);
}

TEST(DeadReckon, ForwardAtQuarterTurnMovesAlongYo) {
    // Heading +90 deg (toward y_o): forward walking goes along y_o.  The step uses the
    // heading at the START of the tick, so set it first with a zero-velocity tick.
    DeadReckon d;
    d.step(0.0, 0.0, M_PI / 2.0, 0.02);
    for (int i = 0; i < 50; ++i) d.step(0.0, 0.2, M_PI / 2.0, 0.02);
    EXPECT_NEAR(d.x(), 0.0, 1e-9);
    EXPECT_NEAR(d.y(), 0.2, 1e-9);
}

TEST(DeadReckon, UnwrappedYawIsKeptAsGiven) {
    DeadReckon d;
    d.step(0.0, 0.0, 3.0 * M_PI, 0.02);
    EXPECT_DOUBLE_EQ(d.yaw(), 3.0 * M_PI);
}

TEST(DeadReckon, ASquareClosesOnItself) {
    // Four 1 m legs with quarter turns between them return to the origin.
    DeadReckon d;
    for (int leg = 0; leg < 4; ++leg) {
        const double psi = leg * M_PI / 2.0;
        d.step(0.0, 0.0, psi, 0.02);
        for (int i = 0; i < 100; ++i) d.step(0.0, 0.5, psi, 0.02);   // 2 s at 0.5 m/s
    }
    EXPECT_NEAR(d.x(), 0.0, 1e-9);
    EXPECT_NEAR(d.y(), 0.0, 1e-9);
    EXPECT_NEAR(d.dist(), 4.0, 1e-9);
}

TEST(DeadReckon, ResetZeroesEverything) {
    DeadReckon d;
    d.step(0.1, 0.2, 0.3, 0.02);
    d.reset();
    EXPECT_EQ(d.x(), 0.0); EXPECT_EQ(d.y(), 0.0); EXPECT_EQ(d.yaw(), 0.0); EXPECT_EQ(d.dist(), 0.0);
}
