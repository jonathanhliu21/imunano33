#ifndef INCLUDE_IMUNANO33TEST_TESTUTIL_HPP_
#define INCLUDE_IMUNANO33TEST_TESTUTIL_HPP_

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <imunano33/simplevectors.hpp>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433
#endif

using namespace svector;

inline void nearCheck(Vector3D a, Vector3D b, double tol = 0.0001) {
  static const std::vector<std::string> comps{"x", "y", "z"};

  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(a[i], b[i], tol) << "failed component: " << comps[i];
  }
}

#endif
