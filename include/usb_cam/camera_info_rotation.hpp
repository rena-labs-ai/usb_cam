#ifndef USB_CAM__CAMERA_INFO_ROTATION_HPP_
#define USB_CAM__CAMERA_INFO_ROTATION_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace usb_cam
{
struct ImageSize
{
  uint32_t width;
  uint32_t height;
};

inline void validate_rotation(int degrees)
{
  switch (degrees) {
    case 0:
    case 90:
    case 180:
    case 270:
      return;
    default:
      throw std::invalid_argument("rotation_degrees must be 0, 90, 180, or 270");
  }
}

inline bool has_intrinsic_matrix(const std::array<double, 9> & matrix)
{
  return std::any_of(matrix.begin(), matrix.end(), [](double value) {
    return value != 0.0;
  });
}

inline bool has_projection_matrix(const std::array<double, 12> & matrix)
{
  return std::any_of(matrix.begin(), matrix.end(), [](double value) {
    return value != 0.0;
  });
}

inline ImageSize rotated_image_size(ImageSize input, int degrees)
{
  validate_rotation(degrees);
  if (degrees == 90 || degrees == 270) {
    return ImageSize{input.height, input.width};
  }
  return input;
}

inline std::array<double, 9> rotated_intrinsic_k(
  const std::array<double, 9> & k,
  ImageSize input,
  int degrees)
{
  validate_rotation(degrees);

  const double fx = k[0];
  const double fy = k[4];
  const double cx = k[2];
  const double cy = k[5];

  switch (degrees) {
    case 0:
      return k;
    case 90:
      return {
        fy, 0.0, static_cast<double>(input.height - 1) - cy,
        0.0, fx, cx,
        0.0, 0.0, 1.0};
    case 180:
      return {
        fx, 0.0, static_cast<double>(input.width - 1) - cx,
        0.0, fy, static_cast<double>(input.height - 1) - cy,
        0.0, 0.0, 1.0};
    case 270:
      return {
        fy, 0.0, cy,
        0.0, fx, static_cast<double>(input.width - 1) - cx,
        0.0, 0.0, 1.0};
    default:
      return k;
  }
}

inline std::array<double, 12> rotated_projection_p(
  const std::array<double, 12> & p,
  ImageSize input,
  int degrees)
{
  validate_rotation(degrees);

  const double fx = p[0];
  const double fy = p[5];
  const double cx = p[2];
  const double cy = p[6];
  const double tx = p[3];
  const double ty = p[7];

  switch (degrees) {
    case 0:
      return p;
    case 90:
      return {
        fy, 0.0, static_cast<double>(input.height - 1) - cy, ty,
        0.0, fx, cx, tx,
        0.0, 0.0, 1.0, 0.0};
    case 180:
      return {
        fx, 0.0, static_cast<double>(input.width - 1) - cx, tx,
        0.0, fy, static_cast<double>(input.height - 1) - cy, ty,
        0.0, 0.0, 1.0, 0.0};
    case 270:
      return {
        fy, 0.0, cy, ty,
        0.0, fx, static_cast<double>(input.width - 1) - cx, tx,
        0.0, 0.0, 1.0, 0.0};
    default:
      return p;
  }
}

inline std::vector<double> rotated_distortion_d(
  std::vector<double> d, const std::string & model, int degrees)
{
  validate_rotation(degrees);
  // Radial terms are rotation-invariant; only the Brown models' tangential p1, p2
  // (d[2], d[3]) turn. Equidistant D is all radial and passes through.
  if ((model != "plumb_bob" && model != "rational_polynomial") || d.size() < 4) {
    return d;
  }
  const double p1 = d[2];
  const double p2 = d[3];
  switch (degrees) {
    case 90:
      d[2] = p2;
      d[3] = -p1;
      break;
    case 180:
      d[2] = -p1;
      d[3] = -p2;
      break;
    case 270:
      d[2] = -p2;
      d[3] = p1;
      break;
    default:
      break;
  }
  return d;
}
}  // namespace usb_cam

#endif  // USB_CAM__CAMERA_INFO_ROTATION_HPP_
