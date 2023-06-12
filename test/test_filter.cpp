#include <gtest/gtest.h>
#include <imunano33/filter.hpp>

#include "testutil.hpp"

using namespace imunano33;
using namespace svector;

TEST(Filter, DefaultConstructor) {
  Filter f;
  Quaternion q{1, Vector3D{}};
  EXPECT_EQ(f.getRotQ(), q);
  EXPECT_NEAR(f.getGyroFavoring(), 0.98, 0.0001);
}

TEST(Filter, FavorConstructorNoClamp) {
  Filter f{0.5};
  EXPECT_NEAR(f.getGyroFavoring(), 0.5, 0.0001);
  Quaternion q{1, Vector3D{}};
  EXPECT_EQ(f.getRotQ(), q);
}

TEST(Filter, FavorConstructorClampHi) {
  Filter f{5};
  EXPECT_NEAR(f.getGyroFavoring(), 1.0, 0.0001);
  Quaternion q{1, Vector3D{}};
  EXPECT_EQ(f.getRotQ(), q);
}

TEST(Filter, FavorConstructorClampLo) {
  Filter f{-5};
  EXPECT_NEAR(f.getGyroFavoring(), 0.0, 0.0001);
  Quaternion q{1, Vector3D{}};
  EXPECT_EQ(f.getRotQ(), q);
}

TEST(Filter, FullConstructorNoClamp) {
  Quaternion q{1.5, Vector3D{1, 0, 3}};
  Filter f{0.5, q};
  EXPECT_NEAR(f.getGyroFavoring(), 0.5, 0.0001);

  Quaternion qN = q.norm();
  EXPECT_NEAR(f.getRotQ().w(), qN.w(), 0.0001);
  nearCheck(f.getRotQ().vec(), qN.vec(), 0.0001);
}

TEST(Filter, FullConstructorClampHi) {
  Quaternion q{-1.5, Vector3D{3, 3, 0}};
  Filter f{5, q};
  EXPECT_NEAR(f.getGyroFavoring(), 1.0, 0.0001);

  Quaternion qN = q.norm();
  EXPECT_NEAR(f.getRotQ().w(), qN.w(), 0.0001);
  nearCheck(f.getRotQ().vec(), qN.vec(), 0.0001);
}

TEST(Filter, FullConstructorClampLo) {
  Quaternion q{2.5, Vector3D{5, 0, 1}};
  Filter f{-5, q};
  EXPECT_NEAR(f.getGyroFavoring(), 0.0, 0.0001);

  Quaternion qN = q.norm();
  EXPECT_NEAR(f.getRotQ().w(), qN.w(), 0.0001);
  nearCheck(f.getRotQ().vec(), qN.vec(), 0.0001);
}

TEST(Filter, CopyConstructor) {
  Quaternion q{2.5, Vector3D{5, 0, 1}};
  Filter f{-5, q};
  Filter f2{f};

  Quaternion qN = q.norm();
  EXPECT_NEAR(f2.getGyroFavoring(), 0.0, 0.0001);
  EXPECT_NEAR(f2.getRotQ().w(), qN.w(), 0.0001);
  nearCheck(f2.getRotQ().vec(), qN.vec(), 0.0001);
}

TEST(Filter, AssignOperator) {
  Quaternion q{2.5, Vector3D{5, 0, 1}};
  Filter f{0.89, q};
  Filter f2{0.3};

  f2 = f;

  Quaternion qN = q.norm();
  EXPECT_NEAR(f2.getGyroFavoring(), 0.89, 0.0001);
  EXPECT_NEAR(f2.getRotQ().w(), qN.w(), 0.0001);
  nearCheck(f2.getRotQ().vec(), qN.vec(), 0.0001);
}

TEST(Filter, Reset) {
  Quaternion q{2.5, Vector3D{5, 0, 1}};
  Filter f{0.89, q};

  f.reset();

  Quaternion qN{1, Vector3D{}};
  EXPECT_NEAR(f.getGyroFavoring(), 0.89, 0.0001);
  EXPECT_NEAR(f.getRotQ().w(), qN.w(), 0.0001);
  nearCheck(f.getRotQ().vec(), qN.vec(), 0.0001);
}

TEST(Filter, SetRotQ) {
  Quaternion q{2.5, Vector3D{5, 0, 1}};
  Filter f{0.89, q};

  Quaternion q2{3, Vector3D{0, 3, 2}};
  f.setRotQ(q2);

  Quaternion qN = q2.norm();
  EXPECT_NEAR(f.getGyroFavoring(), 0.89, 0.0001);
  EXPECT_NEAR(f.getRotQ().w(), qN.w(), 0.0001);
  nearCheck(f.getRotQ().vec(), qN.vec(), 0.0001);
}

TEST(Filter, SetGyroFavoring) {
  Quaternion q{2.5, Vector3D{5, 0, 1}};
  Filter f{0.89, q};

  f.setGyroFavoring(0.98);

  Quaternion qN = q.norm();
  EXPECT_NEAR(f.getGyroFavoring(), 0.98, 0.0001);
  EXPECT_NEAR(f.getRotQ().w(), qN.w(), 0.0001);
  nearCheck(f.getRotQ().vec(), qN.vec(), 0.0001);
}
