#include <gtest/gtest.h>
#include <imunano33/quaternion.hpp>

using namespace imunano33;
using namespace svector;

void nearCheck(Vector3D a, Vector3D b, double tol = 0.0001) {
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(a[i], b[i], tol);
  }
}

TEST(Quaternion, RotateTest) {
  // 90 around x-axis
  Vector3D vec{0, 1, 0};
  Vector3D axis{1, 0, 0};
  Vector3D res = Quaternion::Rotate(vec, axis, M_PI / 2);
  Vector3D expect = {0, 0, 1};
  nearCheck(res, expect);

  // 180 around x-axis
  vec = {0, 1, 0};
  axis = {1, 0, 0};
  res = Quaternion::Rotate(vec, axis, M_PI);
  expect = {0, -1, 0};
  nearCheck(res, expect);

  // test for other axes as well
  vec = {1, 0, 0};
  axis = {0, 1, 0};
  res = Quaternion::Rotate(vec, axis, -M_PI / 4);
  expect = {std::sqrt(2) / 2.0, 0, std::sqrt(2) / 2.0};
  nearCheck(res, expect);
}

TEST(Quaternion, RotateVecTest) {
  Quaternion rotQ{{1, 0, 0}, M_PI / 2};
  auto res = rotQ.Rotate({0, 1, 0});
  Vector3D expect = {0, 0, 1};
  nearCheck(res, expect);
}

TEST(Quaternion, RotateVecNonZeroTest) {
  Quaternion rotQ{{1, 0, 0}, M_PI};
  auto res = rotQ.Rotate({1, 1, 0});
  Vector3D expect = {1, -1, 0};
  nearCheck(res, expect);
}
