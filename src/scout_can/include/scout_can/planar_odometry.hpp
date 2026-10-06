#ifndef SCOUT_CAN__PLANAR_ODOMETRY_HPP_
#define SCOUT_CAN__PLANAR_ODOMETRY_HPP_

#include <array>
#include <cmath>
#include <cstdint>

// Velocity is expressed in base_link; pose and covariance in odom.
struct PlanarOdometry
{
  double x = 0.0, y = 0.0, yaw = 0.0;
  std::array<double, 9> covariance{1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};
  int64_t last_stamp_ns = 0;
  bool initialized = false;
  std::array<double, 3> last_velocity{};

  enum class Update { First, Integrated, Discontinuity, Duplicate };

  Update update(
    int64_t stamp_ns, const std::array<double, 3> & velocity,
    const std::array<double, 3> & variance, double timeout)
  {
    if (initialized && stamp_ns == last_stamp_ns) {
      return Update::Duplicate;
    }
    const double dt = static_cast<double>(stamp_ns - last_stamp_ns) * 1e-9;
    const bool first = !initialized;
    const auto previous = last_velocity;
    last_stamp_ns = stamp_ns;
    last_velocity = velocity;
    initialized = true;
    if (first) {
      return Update::First;
    }
    if (dt <= 0.0 || dt > timeout) {
      // Missing motion is unknown. Keep pose continuous, increase uncertainty.
      const double interval = dt > 0.0 ? dt : timeout;
      const double travel = interval * std::hypot(previous[0], previous[1]);
      covariance[0] += travel * travel + variance[0] * interval * interval;
      covariance[4] += travel * travel + variance[1] * interval * interval;
      covariance[8] += (previous[2] * previous[2] + variance[2]) * interval * interval;
      return Update::Discontinuity;
    }

    const double vx = 0.5 * (previous[0] + velocity[0]);
    const double vy = 0.5 * (previous[1] + velocity[1]);
    const double wz = 0.5 * (previous[2] + velocity[2]);
    const double heading = yaw + 0.5 * wz * dt;
    const double c = std::cos(heading), s = std::sin(heading);
    const double dx = (c * vx - s * vy) * dt;
    const double dy = (s * vx + c * vy) * dt;
    x += dx;
    y += dy;
    yaw = std::remainder(yaw + wz * dt, 2.0 * std::acos(-1.0));

    // First-order uncertainty propagation: P = F P F^T + G Q G^T.
    const double f[3][3] = {{1, 0, -dy}, {0, 1, dx}, {0, 0, 1}};
    const double g[3][3] = {
      {c * dt, -s * dt, -0.5 * dy * dt},
      {s * dt, c * dt, 0.5 * dx * dt}, {0, 0, dt}};
    std::array<double, 9> next{};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          next[i * 3 + j] += g[i][k] * variance[k] * g[j][k];
          for (int l = 0; l < 3; ++l) {
            next[i * 3 + j] += f[i][k] * covariance[k * 3 + l] * f[j][l];
          }
        }
      }
    }
    covariance = next;
    return Update::Integrated;
  }
};

#endif
