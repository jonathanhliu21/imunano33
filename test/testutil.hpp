#ifndef INCLUDE_IMUNANO33TEST_TESTUTIL_HPP_
#define INCLUDE_IMUNANO33TEST_TESTUTIL_HPP_

#include <gtest/gtest.h>
#include <imunano33/simplevectors.hpp>

using namespace svector;

inline void nearCheck(Vector3D a, Vector3D b, double tol = 0.0001) {
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(a[i], b[i], tol);
  }
}

#endif
