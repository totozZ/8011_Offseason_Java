#include <gtest/gtest.h>

#include "shooting/ShotTable.h"

using namespace units::literals;

TEST(ShotTableTest, ClampsHubEndpoints) {
  const auto near = shooting::ShotTable::Hub(0.5_m);
  const auto first = shooting::ShotTable::Hub(1.7_m);
  const auto far = shooting::ShotTable::Hub(8.0_m);
  const auto last = shooting::ShotTable::Hub(4.75_m);

  EXPECT_DOUBLE_EQ(near.flywheel.value(), first.flywheel.value());
  EXPECT_DOUBLE_EQ(near.upperFeeder.value(), first.upperFeeder.value());
  EXPECT_DOUBLE_EQ(far.flywheel.value(), last.flywheel.value());
  EXPECT_DOUBLE_EQ(far.pitch.value(), last.pitch.value());
}

TEST(ShotTableTest, InterpolatesAllThreeHubValues) {
  const auto lower = shooting::ShotTable::Hub(2.0_m);
  const auto upper = shooting::ShotTable::Hub(2.215_m);
  const auto middle = shooting::ShotTable::Hub(2.1075_m);

  EXPECT_NEAR(middle.flywheel.value(),
              (lower.flywheel.value() + upper.flywheel.value()) / 2.0,
              1e-9);
  EXPECT_NEAR(middle.upperFeeder.value(),
              (lower.upperFeeder.value() + upper.upperFeeder.value()) / 2.0,
              1e-9);
  EXPECT_NEAR(middle.pitch.value(),
              (lower.pitch.value() + upper.pitch.value()) / 2.0, 1e-9);
}

TEST(ShotTableTest, TowerUsesThreePointZeroFourEightMeters) {
  const auto tower = shooting::ShotTable::Tower();
  const auto sampled = shooting::ShotTable::Hub(3.048_m);

  EXPECT_DOUBLE_EQ(tower.flywheel.value(), sampled.flywheel.value());
  EXPECT_DOUBLE_EQ(tower.upperFeeder.value(), sampled.upperFeeder.value());
  EXPECT_DOUBLE_EQ(tower.pitch.value(), sampled.pitch.value());
}

TEST(ShotTableTest, MirrorsHubRegionByAlliance) {
  EXPECT_TRUE(shooting::ShotTable::IsHubRegion(
      shooting::AllianceSide::kBlue, 4.9_m));
  EXPECT_FALSE(shooting::ShotTable::IsHubRegion(
      shooting::AllianceSide::kBlue, 5.1_m));
  EXPECT_TRUE(shooting::ShotTable::IsHubRegion(
      shooting::AllianceSide::kRed, 11.6_m));
  EXPECT_FALSE(shooting::ShotTable::IsHubRegion(
      shooting::AllianceSide::kRed, 11.4_m));
  EXPECT_FALSE(shooting::ShotTable::IsHubRegion(
      shooting::AllianceSide::kUnknown, 4.0_m));
}

TEST(ShotTableTest, TimeoutCannotBypassPitchHoming) {
  EXPECT_FALSE(shooting::ShotTable::FeedAllowed(false, true, true));
  EXPECT_TRUE(shooting::ShotTable::FeedAllowed(true, true, false));
  EXPECT_TRUE(shooting::ShotTable::FeedAllowed(true, false, true));
  EXPECT_FALSE(shooting::ShotTable::FeedAllowed(true, false, false));
}

TEST(ShotTableTest, FarPassRaisesFlywheelAndSlowsUpperFeeder) {
  const auto near = shooting::ShotTable::Pass(10.061_m);
  const auto far = shooting::ShotTable::Pass(13.133_m);

  EXPECT_GT(far.flywheel.value(), near.flywheel.value());
  EXPECT_LT(far.upperFeeder.value(), near.upperFeeder.value());
}
