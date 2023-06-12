#include <cmath>

#include <gtest/gtest.h>
#include <imunano33/mathutil.hpp>

using namespace imunano33;
using namespace svector;

TEST(MathUtil, NearZeroNum) {
  double num1 = std::sqrt(4.4105) - std::sqrt(4.4105);
  EXPECT_TRUE(MathUtil::nearZero(num1));
}

TEST(MathUtil, NotNearZeroNum) {
  double num1 = std::sqrt(4.4105) + std::sqrt(4.4105);
  EXPECT_FALSE(MathUtil::nearZero(num1));
}

TEST(MathUtil, NearZeroVec) {
  Vector3D vec1{std::sqrt(4.293781), std::cos(4.492), std::atan(55.2981)};
  Vector3D vec2{std::sqrt(4.293781), std::cos(4.492), std::atan(55.2981)};

  EXPECT_TRUE(MathUtil::nearZero(vec1 - vec2));
}

TEST(MathUtil, NotNearZeroVec) {
  Vector3D vec1{std::sqrt(4.293781), std::cos(4.492), std::atan(55.2981)};
  Vector3D vec2{std::sqrt(4.293781), std::cos(4.492), std::atan(55.2981)};

  EXPECT_FALSE(MathUtil::nearZero(vec1 + vec2));
}

TEST(MathUtil, NearEqNum) {
  double num1 = std::cos(std::sqrt(0.7457));
  double num2 = std::cos(std::sqrt(0.7457));

  EXPECT_TRUE(MathUtil::nearEq(num1, num2));
}

TEST(MathUtil, NearInEqNum) {
  double num1 = std::cos(std::sqrt(0.7457));
  double num2 = std::sin(std::sqrt(0.7457));

  EXPECT_FALSE(MathUtil::nearEq(num1, num2));
}

TEST(MathUtil, ClampLo) {
  EXPECT_NEAR(MathUtil::clamp(-5, 1, 5), 1, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(-3, -2, 8), -2, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(-5, 3, 10), 3, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(-10.0, 1.1, 3.3), 1.1, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(0.999, 1.0, 1.001), 1.0, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(-10000.0, 1.0, 1.0001), 1.0, 0.0001);
}

TEST(MathUtil, ClampHi) {
  EXPECT_NEAR(MathUtil::clamp(5, 1, 5), 5, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(10, -2, 8), 8, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(10.0001, 3.0, 10.0), 10, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(3.3, 1.1, 3.3), 3.3, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(1.002, 1.0, 1.001), 1.001, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(10000.0, 1.0, 1.0001), 1.0001, 0.0001);
}

TEST(MathUtil, ClampNorm) {
  EXPECT_NEAR(MathUtil::clamp(3, 1, 5), 3, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(-2, -2, 8), -2, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(5.5, 3.0, 10.0), 5.5, 0.0001);
  EXPECT_NEAR(MathUtil::clamp(1.101, 1.1, 3.3), 1.101, 0.0001);
}
