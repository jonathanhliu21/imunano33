#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433
#endif

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

TEST(Filter, UpdateGyroXPos90) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {M_PI / 2, 0, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, 0, 1}, 0.0001);
  nearCheck(kRes, {0, -1, 0}, 0.0001);
}

TEST(Filter, UpdateGyroXPos180) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {M_PI, 0, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, -1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, -1}, 0.0001);
}

TEST(Filter, UpdateGyroXPos270) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {3 * M_PI / 2, 0, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  // std::cout << "one" << std::endl;
  nearCheck(iRes, {1, 0, 0}, 0.0001);
  // std::cout << "two" << std::endl;
  nearCheck(jRes, {0, 0, -1}, 0.0001);
  // std::cout << "three" << std::endl;
  nearCheck(kRes, {0, 1, 0}, 0.0001);
}

TEST(Filter, UpdateGyroXPos360) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {M_PI * 2, 0, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroXNeg270) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {-3 * M_PI / 2, 0, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, 0, 1}, 0.0001);
  nearCheck(kRes, {0, -1, 0}, 0.0001);
}

TEST(Filter, UpdateGyroXNeg180) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {-M_PI, 0, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, -1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, -1}, 0.0001);
}

TEST(Filter, UpdateGyroXNeg90) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {-M_PI / 2, 0, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  // std::cout << "one" << std::endl;
  nearCheck(iRes, {1, 0, 0}, 0.0001);
  // std::cout << "two" << std::endl;
  nearCheck(jRes, {0, 0, -1}, 0.0001);
  // std::cout << "three" << std::endl;
  nearCheck(kRes, {0, 1, 0}, 0.0001);
}

TEST(Filter, UpdateGyroXNeg360) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {-M_PI * 2, 0, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroYPos270) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, 3 * M_PI / 2, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {0, 0, 1}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {-1, 0, 0}, 0.0001);
}

TEST(Filter, UpdateGyroYPos180) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, M_PI, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {-1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, -1}, 0.0001);
}

TEST(Filter, UpdateGyroYPos90) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, M_PI / 2, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {0, 0, -1}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {1, 0, 0}, 0.0001);
}

TEST(Filter, UpdateGyroYPos360) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  // rotate +90
  Filter f{1};
  f.update({}, {0, M_PI * 2, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroYNeg90) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, -M_PI / 2, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {0, 0, 1}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {-1, 0, 0}, 0.0001);
}

TEST(Filter, UpdateGyroYNeg180) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, -M_PI, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {-1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, -1}, 0.0001);
}

TEST(Filter, UpdateGyroYNeg270) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, -3 * M_PI / 2, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {0, 0, -1}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {1, 0, 0}, 0.0001);
}

TEST(Filter, UpdateGyroYNeg360) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, -M_PI * 2, 0}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroZPos270) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, 0, 3 * M_PI / 2}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {0, -1, 0}, 0.0001);
  nearCheck(jRes, {1, 0, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroZPos180) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, 0, M_PI / 2}, 2);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {-1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, -1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroZPos90) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, 0, M_PI / 6}, 3);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {0, 1, 0}, 0.0001);
  nearCheck(jRes, {-1, 0, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroZPos360) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  // rotate +90
  Filter f{1};
  f.update({}, {0, M_PI * 4, 0}, 0.5);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroZNeg90) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, 0, -M_PI / 2}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {0, -1, 0}, 0.0001);
  nearCheck(jRes, {1, 0, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroZNeg180) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, 0, -M_PI}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {-1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, -1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroZNeg270) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, 0, -3 * M_PI / 2}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {0, 1, 0}, 0.0001);
  nearCheck(jRes, {-1, 0, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

TEST(Filter, UpdateGyroZNeg360) {
  Vector3D i{1, 0, 0};
  Vector3D j{0, 1, 0};
  Vector3D k{0, 0, 1};

  Filter f{1};
  f.update({}, {0, 0, -M_PI * 2}, 1);
  Quaternion q = f.getRotQ();

  Vector3D iRes = q.rotate(i);
  Vector3D jRes = q.rotate(j);
  Vector3D kRes = q.rotate(k);

  nearCheck(iRes, {1, 0, 0}, 0.0001);
  nearCheck(jRes, {0, 1, 0}, 0.0001);
  nearCheck(kRes, {0, 0, 1}, 0.0001);
}

// TEST(Filter, UpdateGyroY) { Filter f{1}; }

// TEST(Filter, UpdateGyroZ) { Filter f{1}; }

// TEST(Filter, UpdateAccel) {}
