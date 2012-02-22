CMAKE_MINIMUM_REQUIRED( VERSION 2.8.5 )

set(PACKAGE_NAME easykf)
set(PACKAGE_VERSION 2.03)

# Adds the path to the FindGSL.cmake
SET( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
                       "${CMAKE_MODULE_PATH}" )

# Get required and optional library
FIND_PACKAGE( GSL REQUIRED )

include_directories( ${GSL_INCLUDE_DIR})
link_directories(${GSL_LINK_DIRECTORIES})

add_subdirectory (src)
add_subdirectory (examples)

