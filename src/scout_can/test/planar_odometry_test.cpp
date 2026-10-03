#include <cassert>
#include "scout_can/planar_odometry.hpp"

int main()
{
  const std::array<double, 3> noise{0.0009, 0.0036, 0.0009};
  PlanarOdometry state;
  assert(state.update(1000000000, {0, 0.5, 0}, noise, 0.2) ==
    PlanarOdometry::Update::First);
  // Queued frames retain their reception intervals even when processed together.
  for (int i = 1; i <= 5; ++i) {
    assert(state.update(1000000000 + i * 20000000, {0, 0.5, 0}, noise, 0.2) ==
      PlanarOdometry::Update::Integrated);
  }
  assert(std::abs(state.y - 0.05) < 1e-12);
  assert(state.covariance[4] > state.covariance[0]);
  const double y = state.y;
  assert(state.update(1100000000, {0, 5, 0}, noise, 0.2) ==
    PlanarOdometry::Update::Duplicate);
  assert(state.last_velocity[1] == 0.5);
  assert(state.update(2100000000, {0, 0.5, 0}, noise, 0.2) ==
    PlanarOdometry::Update::Discontinuity);
  assert(state.y == y);
  assert(state.covariance[4] > 0.25);
  state.update(2200000000, {0, 0.5, 0}, noise, 0.2);
  assert(std::abs(state.y - 0.10) < 1e-12);
  assert(state.update(2000000000, {0, 0.5, 0}, noise, 0.2) ==
    PlanarOdometry::Update::Discontinuity);
  assert(std::abs(state.y - 0.10) < 1e-12);
  state.update(2100000000, {0, 0.5, 0}, noise, 0.2);
  assert(std::abs(state.y - 0.15) < 1e-12);

  PlanarOdometry rotated;
  rotated.yaw = std::acos(-1.0) / 2;
  rotated.update(1000000000, {0, 0.5, 0}, noise, 0.2);
  rotated.update(1100000000, {0, 0.5, 0}, noise, 0.2);
  assert(std::abs(rotated.x + 0.05) < 1e-12);
  assert(std::abs(rotated.y) < 1e-12);

  PlanarOdometry turning;
  turning.update(1000000000, {1, 0, 1}, noise, 0.2);
  turning.update(1100000000, {1, 0, 1}, noise, 0.2);
  assert(std::abs(turning.x - 0.1 * std::cos(0.05)) < 1e-12);
  assert(std::abs(turning.y - 0.1 * std::sin(0.05)) < 1e-12);
  assert(std::abs(turning.yaw - 0.1) < 1e-12);
  for (int i = 0; i < 3; ++i) {
    assert(turning.covariance[i * 3 + i] > 0);
    for (int j = 0; j < 3; ++j) {
      assert(std::abs(turning.covariance[i * 3 + j] -
        turning.covariance[j * 3 + i]) < 1e-12);
    }
  }

  PlanarOdometry accelerating;
  accelerating.update(1000000000, {0, 0, 0}, noise, 0.2);
  accelerating.update(1100000000, {1, 0, 0}, noise, 0.2);
  assert(std::abs(accelerating.x - 0.05) < 1e-12);
}
