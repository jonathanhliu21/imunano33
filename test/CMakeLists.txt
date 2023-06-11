message("-- Building tests")

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# don't install gtest when running `make install`
set(INSTALL_GTEST OFF CACHE BOOL "Disable installation of googletest." FORCE)

add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)

include_directories(../include/)

add_executable(
  test_all
  test_all.cpp
)
target_link_libraries(
  test_all
  PRIVATE
  GTest::GTest
)

include(GoogleTest)
gtest_discover_tests(test_all)
