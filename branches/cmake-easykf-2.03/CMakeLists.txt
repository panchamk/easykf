CMAKE_MINIMUM_REQUIRED( VERSION 2.8.4 )

project (easykf) 

set(EASYKF_VERSION "2.3")
set(PACKAGE_VERSION ${EASYKF_VERSION})

# Adds the path to the FindGSL.cmake
SET( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
                       "${CMAKE_MODULE_PATH}" )

###################################
# Get required and optional library
###################################

FIND_PACKAGE( GSL REQUIRED )

include_directories( ${GSL_INCLUDE_DIR})
link_directories(${GSL_LINK_DIRECTORIES})

###################################
# Some prefix definitions
###################################

# Define the prefix where libs are going to be installed
SET (LIB_SUFFIX "" CACHE STRING "Define suffix of directory name (32/64)" )
SET (LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}" CACHE STRING "Library directory name")

# Define the path to install the headers
SET(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include/easykf/" CACHE PATH "The subdirectory to the header prefix")

# Define the path to install the documentation
SET(DOC_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/share/doc/easykf" CACHE PATH "The subdirectory to the documentation prefix")

# Define the path to install the pc pkg-config file
SET(PKGCONFIG_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/pkgconfig/" CACHE STRING "Base directory for pkgconfig files")

###################################
# Pkg-config stuff
###################################
# Let's replace the macros of the pc file
# The @ONLY means replacing only the @VAR@ variables, not the for example ${VAR}
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/easykf.pc.cmake 
               ${CMAKE_CURRENT_BINARY_DIR}/easykf.pc @ONLY)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/easykf.pc
    	DESTINATION ${PKGCONFIG_INSTALL_DIR})


###################################
#  Subdirectories
###################################
add_subdirectory (src)
add_subdirectory (examples)
add_subdirectory (doc)


###################################
# For building packages
###################################

# set(CPACK_PACKAGE_NAME ${CMAKE_PROJECT_NAME})
# set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "C++ Library for Kalman filtering")
# set(CPACK_PACKAGE_VENDOR "Jeremy Fix")
# set(CPACK_PACKAGE_VERSION ${PACKAGE_VERSION})
# SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README")
# SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

# SET(CPACK_RPM_PACKAGE_SUMMARY 	${CPACK_PACKAGE_DESCRIPTION_SUMMARY})
# SET(CPACK_RPM_PACKAGE_NAME 	${CMAKE_PROJECT_NAME})
# SET(CPACK_RPM_PACKAGE_VERSION  ${CPACK_PACKAGE_VERSION})
# SET(CPACK_RPM_PACKAGE_RELEASE 	1)
# SET(CPACK_RPM_PACKAGE_LICENSE 	"New BSD")
# SET(CPACK_RPM_PACKAGE_GROUP 	"Development/Libraries")
# SET(CPACK_RPM_PACKAGE_VENDOR 	${CPACK_PACKAGE_VENDOR})
# SET(CPACK_RPM_PACKAGE_DESCRIPTION "C++ Library for Kalman Filtering")
# set(CPACK_RPM_PACKAGE_REQUIRES "gsl >= 1.14, cmake >= 2.8.4")

# SET(CPACK_GENERATOR "RPM" "TGZ")
 
# INCLUDE(CPack)
