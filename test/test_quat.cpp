#include <gtest/gtest.h>
#include <imunano33/quaternion.hpp>

#include "testutil.hpp"

using namespace imunano33;
using namespace svector;

// if rotate works, then rotation constructor, w(), vec(), magn(), conj(),
// inv() works
TEST(Quaternion, RotateTest) {
  // 90 around x-axis
  Vector3D vec{0, 1, 0};
  Vector3D axis{1, 0, 0};
  Vector3D res = Quaternion::rotate(vec, axis, M_PI / 2);
  Vector3D expect = {0, 0, 1};
  nearCheck(res, expect);

  // 180 around x-axis
  vec = {0, 1, 0};
  axis = {1, 0, 0};
  res = Quaternion::rotate(vec, axis, M_PI);
  expect = {0, -1, 0};
  nearCheck(res, expect);

  // test for other axes as well
  vec = {1, 0, 0};
  axis = {0, 1, 0};
  res = Quaternion::rotate(vec, axis, -M_PI / 4);
  expect = {std::sqrt(2) / 2.0, 0, std::sqrt(2) / 2.0};
  nearCheck(res, expect);
}

TEST(Quaternion, RotateVecTest) {
  Quaternion rotQ{{1, 0, 0}, M_PI / 2};
  auto res = rotQ.rotate({0, 1, 0});
  Vector3D expect = {0, 0, 1};
  nearCheck(res, expect);
}

TEST(Quaternion, RotateVecNonZeroTest) {
  Quaternion rotQ{{1, 0, 0}, M_PI};
  auto res = rotQ.rotate({1, 1, 0});
  Vector3D expect = {1, -1, 0};
  nearCheck(res, expect);
}

TEST(Quaternion, NormTest) {
  Quaternion q{3, {4.4, 1, 5.1}};
  q = q.norm();

  EXPECT_NEAR(q.w(), 0.403166, 0.0001);
  nearCheck(q.vec(), {0.59131, 0.134389, 0.685382});
}

TEST(Quaternion, EqTest) {
  Quaternion q1{1, {1, 1, 1}};
  Quaternion q2{1, {1, 1, 1}};

  EXPECT_TRUE(q1 == q2);
  EXPECT_FALSE(q1 != q2);
}

TEST(Quaternion, IneqTest) {
  Quaternion q1{2, {1, 0.4, 1}};
  Quaternion q2{1, {1, 1, 1}};

  EXPECT_TRUE(q1 != q2);
  EXPECT_FALSE(q1 == q2);
}

TEST(Quaternion, CopyTest) {
  Quaternion a{3, {1, 2, 4}};
  Quaternion b{a};

  EXPECT_NEAR(b.w(), 3, 0.0001);
  nearCheck(b.vec(), {1, 2, 4}, 0.0001);
}

TEST(Quaternion, AssignTest) {
  Quaternion a{3, {1, 2, 4}};
  Quaternion b{4, {0, 1, 1}};

  b = a;

  EXPECT_NEAR(b.w(), 3, 0.0001);
  nearCheck(b.vec(), {1, 2, 4}, 0.0001);
}
