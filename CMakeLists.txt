cmake_minimum_required(VERSION 3.9)

# Set a name and a version number for your project:
project(imunano33 VERSION 0.0.1 LANGUAGES CXX)

# Initialize some default paths
include(GNUInstallDirs)

# Define the minimum C++ standard that is required
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



# Compilation options
option(IMUNANO33_BUILD_DOCS "Enable building of documentation" OFF)
option(IMUNANO33_BUILD_TESTING "Enable building tests" OFF)

# Add an interface target for our header-only library
add_library(imunano33 INTERFACE)
target_include_directories(imunano33 INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# compile the tests
include(CTest)
if(IMUNANO33_BUILD_TESTING)
  # don't install gtest when running `make install`
  set(INSTALL_GTEST OFF CACHE BOOL "Disable installation of googletest." FORCE)
  enable_testing()
  add_subdirectory(test)
endif()

if(IMUNANO33_BUILD_DOCS)
  # Add the documentation
  add_subdirectory(doc)
endif()

# Add an alias target for use if this project is included as a subproject in another project
add_library(imunano33::imunano33 ALIAS imunano33)

# Install targets and configuration
install(
  TARGETS imunano33
  EXPORT imunano33-config
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  EXPORT imunano33-config
  NAMESPACE imunano33::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/imunano33
)

install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# This prints a summary of found dependencies
include(FeatureSummary)
feature_summary(WHAT ALL)
