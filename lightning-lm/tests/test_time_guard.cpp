#include <gtest/gtest.h>

#include <limits>

#include "common/time_guard.h"

TEST(TimeGuard, AcceptsIncreasingTimestamp) {
    const auto result = lightning::NormalizeLidarTimestamp(2.0, 1.9);
    EXPECT_TRUE(result.accepted);
    EXPECT_FALSE(result.clamped);
    EXPECT_DOUBLE_EQ(result.timestamp, 2.0);
}

TEST(TimeGuard, ClampsOfficialM20BagStyleRollback) {
    const auto result = lightning::NormalizeLidarTimestamp(1.9968, 2.0);
    EXPECT_TRUE(result.accepted);
    EXPECT_TRUE(result.clamped);
    EXPECT_NEAR(result.rollback, 0.0032, 1e-12);
    EXPECT_GT(result.timestamp, 2.0);
}

TEST(TimeGuard, RejectsLargeClockReset) {
    const auto result = lightning::NormalizeLidarTimestamp(1.0, 2.0);
    EXPECT_FALSE(result.accepted);
    EXPECT_FALSE(result.clamped);
}

TEST(TimeGuard, RejectsNonFiniteTimestamp) {
    const auto result = lightning::NormalizeLidarTimestamp(
        std::numeric_limits<double>::quiet_NaN(), 2.0);
    EXPECT_FALSE(result.accepted);
}
