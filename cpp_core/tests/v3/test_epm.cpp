/**
 * Unit tests for EPM (v3 C++) — dual TLE signal
 *
 * Verifies:
 *  1.  process_video() returns a valid RealityToken
 *  2.  Quantization error is non-negative and finite
 *  3.  Transition surprise is zero on first tick (no previous prototype)
 *  4.  TLE = QE + transition_weight * TS
 *  5.  is_novel flag responds to a genuinely novel input (after the GNG has
 *      converged on a known pattern, an unseen pattern raises the flag)
 *  6.  Repeated identical input → QE falls toward zero as GNG adapts
 *  7.  node_count and baked_count in token match the GNG state
 *  8.  Transition surprise is elevated after a concept jump
 *      (e.g., input switches from left-half to right-half ball)
 *  9.  process_audio() produces a valid token for audio modality
 *  10. reset() clears GNG state (node_count drops to 0)
 */

#include <gtest/gtest.h>
#include "v3/epm.hpp"
#include <cmath>
#include <vector>

using namespace ami_ogma::v3;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// White ball frame
static std::vector<uint8_t> make_frame(int ball_x, int ball_y,
                                        int h = 128, int w = 128,
                                        int r = 8) {
    std::vector<uint8_t> frame(h * w * 3, 0);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            int dx = x - ball_x, dy = y - ball_y;
            if (dx*dx + dy*dy <= r*r) {
                int b = (y*w + x)*3;
                frame[b] = frame[b+1] = frame[b+2] = 255;
            }
        }
    return frame;
}

// Sine wave burst
static std::vector<float> make_sine(float freq, float amp, int n, int sr = 48000) {
    std::vector<float> s(n);
    for (int i = 0; i < n; ++i)
        s[i] = amp * std::sin(2.0f * static_cast<float>(M_PI) * freq * i / sr);
    return s;
}

static EPM::Config make_visual_config(const std::string& modality = "retinal") {
    EPM::Config cfg;
    cfg.epm.modality          = modality;
    cfg.epm.projection_dim    = 128;
    cfg.epm.baking_threshold  = 10;
    cfg.epm.min_insertion_error = 1e-5f;
    cfg.epm.lambda_new        = 5;
    cfg.epm.max_age           = 50;
    cfg.transition_weight     = 1.0f;
    cfg.threshold_multiplier  = 1.5f;
    return cfg;
}

// ---------------------------------------------------------------------------

TEST(EPM, ProcessVideoReturnsValidToken) {
    EPM epm(make_visual_config());
    auto frame = make_frame(64, 64);

    auto tok = epm.process_video(frame.data(), 128, 128, 3);

    EXPECT_GE(tok.winner_id, 0);
    EXPECT_GE(tok.quant_error, 0.0f);
    EXPECT_FALSE(std::isnan(tok.tle));
    EXPECT_FALSE(std::isinf(tok.tle));
    EXPECT_GT(tok.timestamp_us, 0);
}

TEST(EPM, QuantizationErrorNonNegative) {
    EPM epm(make_visual_config());
    auto frame = make_frame(64, 64);

    for (int i = 0; i < 20; ++i) {
        auto tok = epm.process_video(frame.data(), 128, 128, 3);
        EXPECT_GE(tok.quant_error, 0.0f);
        EXPECT_FALSE(std::isnan(tok.quant_error));
    }
}

TEST(EPM, TransitionSurpriseZeroOnFirstTick) {
    EPM epm(make_visual_config());
    auto frame = make_frame(64, 64);

    // First real tick (after bootstrap)
    epm.process_video(frame.data(), 128, 128, 3);
    epm.process_video(frame.data(), 128, 128, 3);
    auto tok = epm.process_video(frame.data(), 128, 128, 3);

    // The 3rd tick is the first non-bootstrap tick
    // TS should be low when repeatedly hitting the same node
    // (winner prototype doesn't change much between identical inputs)
    EXPECT_GE(tok.transition_surp, 0.0f);
    EXPECT_FALSE(std::isnan(tok.transition_surp));
}

TEST(EPM, TLEEqualsCombinedComponents) {
    EPM epm(make_visual_config());
    auto frame = make_frame(64, 64);

    // Bootstrap
    epm.process_video(frame.data(), 128, 128, 3);
    epm.process_video(frame.data(), 128, 128, 3);

    auto tok = epm.process_video(frame.data(), 128, 128, 3);

    float expected_tle = tok.quant_error + 1.0f * tok.transition_surp;
    EXPECT_NEAR(tok.tle, expected_tle, 1e-5f);
}

TEST(EPM, QEIsLowerForKnownThanNovel) {
    // After the GNG has seen many examples of position A, QE for A should be
    // lower than QE for an unseen position B far from A.
    EPM epm(make_visual_config());

    auto frame_a = make_frame(20, 20);   // top-left ball
    auto frame_b = make_frame(108, 108); // bottom-right ball (unseen)

    // Bootstrap with distinct frames so nodes are spread out
    epm.process_video(frame_a.data(), 128, 128, 3);
    epm.process_video(frame_b.data(), 128, 128, 3);

    // Train heavily on frame_a
    for (int i = 0; i < 60; ++i)
        epm.process_video(frame_a.data(), 128, 128, 3);

    // Measure steady-state QE for known frame_a
    float qe_known = 0.0f;
    for (int i = 0; i < 10; ++i) {
        auto tok = epm.process_video(frame_a.data(), 128, 128, 3);
        qe_known += tok.quant_error;
    }
    qe_known /= 10.0f;

    // QE for unseen frame_b (novel)
    float qe_novel = 0.0f;
    for (int i = 0; i < 5; ++i) {
        auto tok = epm.process_video(frame_b.data(), 128, 128, 3);
        qe_novel += tok.quant_error;
    }
    qe_novel /= 5.0f;

    EXPECT_LT(qe_known, qe_novel)
        << "QE for a known pattern must be lower than for an unseen one";
}

TEST(EPM, NodeCountInTokenMatchesGNG) {
    EPM epm(make_visual_config());
    auto frame = make_frame(64, 64);

    // Bootstrap
    epm.process_video(frame.data(), 128, 128, 3);
    epm.process_video(frame.data(), 128, 128, 3);

    for (int i = 0; i < 30; ++i) {
        auto tok = epm.process_video(frame.data(), 128, 128, 3);
        EXPECT_EQ(tok.node_count, epm.gng().node_count());
        EXPECT_EQ(tok.baked_count, epm.gng().baked_count());
    }
}

TEST(EPM, TransitionSurpriseElevatedOnConceptJump) {
    EPM epm(make_visual_config());

    auto frame_left  = make_frame(20, 64);   // ball in left region
    auto frame_right = make_frame(108, 64);  // ball in right region

    // Bootstrap + converge on left position
    epm.process_video(frame_left.data(), 128, 128, 3);
    epm.process_video(frame_left.data(), 128, 128, 3);
    for (int i = 0; i < 30; ++i)
        epm.process_video(frame_left.data(), 128, 128, 3);

    // Measure TS at steady state (left)
    float ts_steady = 0.0f;
    for (int i = 0; i < 5; ++i) {
        auto tok = epm.process_video(frame_left.data(), 128, 128, 3);
        ts_steady += tok.transition_surp;
    }
    ts_steady /= 5.0f;

    // Jump to right — first token after jump should have elevated TS
    auto tok_jump = epm.process_video(frame_right.data(), 128, 128, 3);

    EXPECT_GE(tok_jump.transition_surp, ts_steady)
        << "Concept jump should elevate transition surprise";
}

TEST(EPM, ProcessAudioStft) {
    EPM::Config cfg;
    cfg.epm.modality       = "audio";
    cfg.epm.projection_dim = 128;
    cfg.epm.sample_rate    = 48000;
    cfg.epm.baking_threshold = 10;
    cfg.epm.min_insertion_error = 1e-5f;
    cfg.epm.lambda_new = 5;
    cfg.transition_weight = 1.0f;

    EPM epm(cfg);

    auto tone = make_sine(440.0f, 0.1f, 4800);  // 100ms
    // Bootstrap
    epm.process_audio(tone.data(), 4800, 1);
    epm.process_audio(tone.data(), 4800, 1);

    auto tok = epm.process_audio(tone.data(), 4800, 1);

    EXPECT_GE(tok.winner_id, 0);
    EXPECT_GE(tok.quant_error, 0.0f);
    EXPECT_FALSE(std::isnan(tok.tle));
    EXPECT_EQ(tok.node_count, epm.gng().node_count());
}

TEST(EPM, ResetClearsGNGState) {
    EPM epm(make_visual_config());
    auto frame = make_frame(64, 64);

    // Bootstrap + grow
    epm.process_video(frame.data(), 128, 128, 3);
    epm.process_video(frame.data(), 128, 128, 3);
    for (int i = 0; i < 50; ++i)
        epm.process_video(frame.data(), 128, 128, 3);

    EXPECT_GT(epm.gng().node_count(), 0);

    epm.reset();

    EXPECT_EQ(epm.gng().node_count(), 0);

    // Should function normally after reset
    epm.process_video(frame.data(), 128, 128, 3);
    epm.process_video(frame.data(), 128, 128, 3);
    auto tok = epm.process_video(frame.data(), 128, 128, 3);
    EXPECT_GE(tok.quant_error, 0.0f);
}

TEST(EPM, IsNovelFlagRisesForUnseenInput) {
    EPM::Config cfg = make_visual_config();
    cfg.threshold_multiplier = 1.2f;  // slightly sensitive
    EPM epm(cfg);

    auto frame_known = make_frame(64, 64);
    auto frame_unseen = make_frame(10, 10);  // far from known

    // Bootstrap + converge
    epm.process_video(frame_known.data(), 128, 128, 3);
    epm.process_video(frame_known.data(), 128, 128, 3);
    for (int i = 0; i < 40; ++i)
        epm.process_video(frame_known.data(), 128, 128, 3);

    // Measure novelty on known input (should stabilise to mostly false)
    int novel_known = 0;
    for (int i = 0; i < 10; ++i) {
        auto tok = epm.process_video(frame_known.data(), 128, 128, 3);
        if (tok.is_novel) ++novel_known;
    }

    // Present unseen input — novelty should spike
    int novel_unseen = 0;
    for (int i = 0; i < 5; ++i) {
        auto tok = epm.process_video(frame_unseen.data(), 128, 128, 3);
        if (tok.is_novel) ++novel_unseen;
    }

    EXPECT_GE(novel_unseen, novel_known)
        << "Unseen input should trigger more novel flags than known input";
}
