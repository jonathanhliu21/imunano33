#include <gtest/gtest.h>
#include <imunano33/imunano33.hpp>

#include "testutil.hpp"

using namespace imunano33;

TEST(IMUNano33, TestUpdateClim) {
  IMUNano33 proc;
  EXPECT_FALSE(proc.climateDataExists());
  proc.updateClimate(46.5, 46.5, 46.5);

  double res;
  // testing climate
  res = proc.getTemperature<FAHRENHEIT>();
  EXPECT_NEAR(res, 115.7, 0.0001);
  res = proc.getTemperature<CELSIUS>();
  EXPECT_NEAR(res, 46.5, 0.0001);

  res = proc.getPressure<KPA>();
  EXPECT_NEAR(res, 46.5, 0.0001);
  res = proc.getPressure<PSI>();
  EXPECT_NEAR(res, 6.7442548, 0.0001);

  res = proc.getHumidity();
  EXPECT_NEAR(res, 46.5, 0.0001);

  EXPECT_TRUE(proc.climateDataExists());
}

TEST(IMUNano33, TestUpdateIMU) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  IMUNano33 proc;
  proc.updateIMU({0, 0, -1}, {0, 0, M_PI}, 1);

  Quaternion q = proc.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {-1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, -1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(IMUNano33, TestUpdateBoth) {
  IMUNano33 proc;
  proc.update({0, 0, -1}, {0, 0, M_PI}, 1, 46.5, 46.5, 46.5);

  double res;
  // testing climate
  res = proc.getTemperature<FAHRENHEIT>();
  EXPECT_NEAR(res, 115.7, 0.0001);
  res = proc.getTemperature<CELSIUS>();
  EXPECT_NEAR(res, 46.5, 0.0001);

  res = proc.getPressure<KPA>();
  EXPECT_NEAR(res, 46.5, 0.0001);
  res = proc.getPressure<PSI>();
  EXPECT_NEAR(res, 6.7442548, 0.0001);

  res = proc.getHumidity();
  EXPECT_NEAR(res, 46.5, 0.0001);

  EXPECT_TRUE(proc.climateDataExists());

  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Quaternion q = proc.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {-1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, -1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(IMUNano33, TestResetIMU) {
  Quaternion initQ{2, {3, 5, 5}};

  IMUNano33 proc{0.98, initQ};
  proc.update({0, 0, -1}, {0, 0, M_PI}, 1, 46.5, 46.5, 46.5);

  proc.resetIMU();

  Quaternion qN = initQ.norm();
  Quaternion res = proc.getRotQ();
  EXPECT_NEAR(res.w(), qN.w(), 0.0001);
  nearCheck(qN.vec(), res.vec());
}

TEST(IMUNano33, TestZeroIMU) {
  Quaternion initQ{2, {3, 5, 5}};

  IMUNano33 proc{0.98, initQ};
  proc.update({0, 0, -1}, {0, 0, M_PI}, 1, 46.5, 46.5, 46.5);

  proc.zeroIMU();

  Quaternion qN{1, Vector3D{}};
  Quaternion res = proc.getRotQ();
  EXPECT_NEAR(res.w(), qN.w(), 0.0001);
  nearCheck(qN.vec(), res.vec());
}

TEST(IMUNano33, TestResetClimate) {
  Quaternion initQ{2, {3, 5, 5}};

  IMUNano33 proc{0.98, initQ};
  proc.update({0, 0, -1}, {0, 0, M_PI}, 1, 46.5, 46.5, 46.5);

  EXPECT_TRUE(proc.climateDataExists());

  proc.resetClimate();

  EXPECT_FALSE(proc.climateDataExists());
}
