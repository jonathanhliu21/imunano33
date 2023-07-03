find_package(Doxygen REQUIRED)
set(
  DOXYGEN_EXCLUDE_PATTERNS
  "${PROJECT_SOURCE_DIR}/ext/*"
  "${CMAKE_SOURCE_DIR}/build/*"
  "${CMAKE_SOURCE_DIR}/example/*"
  "${CMAKE_SOURCE_DIR}/script/*"
  "${CMAKE_SOURCE_DIR}/test/*"
  "${PROJECT_SOURCE_DIR}/README.md"
  "${PROJECT_SOURCE_DIR}/doc/doxygen-awesome-css/docs"
  "${PROJECT_SOURCE_DIR}/doc/doxygen-awesome-css/include"
  "${PROJECT_SOURCE_DIR}/doc/doxygen-awesome-css/README.md"
)
set(DOXYGEN_SHORT_NAMES YES)
set(DOXYGEN_USE_MATHJAX YES)
set(DOXYGEN_IMAGE_PATH "${PROJECT_SOURCE_DIR}/doc/static/")
set(DOXYGEN_GENERATE_TREEVIEW YES)
set(DOXYGEN_DISABLE_INDEX NO)
set(DOXYGEN_FULL_SIDEBAR NO)
set(DOXYGEN_HTML_EXTRA_STYLESHEET "${CMAKE_SOURCE_DIR}/doc/doxygen-awesome-css/doxygen-awesome.css")
set(DOXYGEN_HTML_COLORSTYLE "LIGHT")

doxygen_add_docs(doxygen
  ${PROJECT_SOURCE_DIR}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT Building doxygen documentation...
)
