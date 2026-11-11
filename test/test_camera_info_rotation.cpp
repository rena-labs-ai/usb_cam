#include <gtest/gtest.h>

#include <array>
#include <stdexcept>
#include <vector>

#include "usb_cam/camera_info_rotation.hpp"

namespace
{
const usb_cam::ImageSize kInput{1280, 720};
const std::array<double, 9> kK{
  100.0, 0.0, 300.0,
  0.0, 200.0, 250.0,
  0.0, 0.0, 1.0};
const std::array<double, 12> kP{
  100.0, 0.0, 300.0, 7.0,
  0.0, 200.0, 250.0, 11.0,
  0.0, 0.0, 1.0, 0.0};
const std::vector<double> kD{-0.3, 0.1, 0.004, -0.006, 0.02};
}  // namespace

TEST(CameraInfoRotation, quarter_turns_swap_the_image_size)
{
  const auto size = usb_cam::rotated_image_size(kInput, 90);
  EXPECT_EQ(size.width, 720u);
  EXPECT_EQ(size.height, 1280u);
}

TEST(CameraInfoRotation, clockwise_quarter_turn_moves_the_principal_point)
{
  const auto k = usb_cam::rotated_intrinsic_k(kK, kInput, 90);
  EXPECT_DOUBLE_EQ(k[0], 200.0);
  EXPECT_DOUBLE_EQ(k[2], 469.0);
  EXPECT_DOUBLE_EQ(k[4], 100.0);
  EXPECT_DOUBLE_EQ(k[5], 300.0);

  const auto p = usb_cam::rotated_projection_p(kP, kInput, 90);
  EXPECT_DOUBLE_EQ(p[0], 200.0);
  EXPECT_DOUBLE_EQ(p[2], 469.0);
  EXPECT_DOUBLE_EQ(p[3], 11.0);
  EXPECT_DOUBLE_EQ(p[5], 100.0);
  EXPECT_DOUBLE_EQ(p[6], 300.0);
  EXPECT_DOUBLE_EQ(p[7], 7.0);
}

TEST(CameraInfoRotation, half_turn_mirrors_the_principal_point)
{
  const auto k = usb_cam::rotated_intrinsic_k(kK, kInput, 180);
  EXPECT_DOUBLE_EQ(k[0], 100.0);
  EXPECT_DOUBLE_EQ(k[2], 979.0);
  EXPECT_DOUBLE_EQ(k[4], 200.0);
  EXPECT_DOUBLE_EQ(k[5], 469.0);
}

TEST(CameraInfoRotation, only_brown_tangential_terms_turn)
{
  const auto d_90 = usb_cam::rotated_distortion_d(kD, "plumb_bob", 90);
  EXPECT_DOUBLE_EQ(d_90[0], -0.3);
  EXPECT_DOUBLE_EQ(d_90[2], -0.006);
  EXPECT_DOUBLE_EQ(d_90[3], -0.004);
  EXPECT_DOUBLE_EQ(d_90[4], 0.02);
  const auto d_180 = usb_cam::rotated_distortion_d(kD, "plumb_bob", 180);
  EXPECT_DOUBLE_EQ(d_180[2], -0.004);
  EXPECT_DOUBLE_EQ(d_180[3], 0.006);
  const auto d_270 = usb_cam::rotated_distortion_d(kD, "plumb_bob", 270);
  EXPECT_DOUBLE_EQ(d_270[2], 0.006);
  EXPECT_DOUBLE_EQ(d_270[3], 0.004);
  EXPECT_TRUE(usb_cam::rotated_distortion_d({}, "", 90).empty());
  const std::vector<double> fisheye{0.1, -0.02, 0.003, -0.004};
  EXPECT_EQ(usb_cam::rotated_distortion_d(fisheye, "equidistant", 90), fisheye);
}

TEST(CameraInfoRotation, rejects_a_rotation_that_is_not_a_quarter_turn)
{
  EXPECT_THROW(usb_cam::rotated_image_size(kInput, 45), std::invalid_argument);
}
