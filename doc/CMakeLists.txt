find_package(Doxygen REQUIRED)
set(
  DOXYGEN_EXCLUDE_PATTERNS
  "${PROJECT_SOURCE_DIR}/ext/*"
  "${CMAKE_SOURCE_DIR}/build/*"
  "${CMAKE_SOURCE_DIR}/example/*"
  "${PROJECT_SOURCE_DIR}/README.md"
)
set(DOXYGEN_SHORT_NAMES YES)
set(DOXYGEN_USE_MATHJAX YES)

doxygen_add_docs(doxygen
  ${PROJECT_SOURCE_DIR}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT Building doxygen documentation...
)
