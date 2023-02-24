############################################################################
# libmix4sam - Mixtures for Smoothing and Mapping Library
#
# Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
# For more information see https://mytuc.org/mix
#
# libmix4sam is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# libmix4sam is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software.  If not, see <http://www.gnu.org/licenses/>.
#
# Contact Information: Sven Lange (sven.lange@etit.tu-chemnitz.de)
############################################################################

# Main CMake file, based on gtsam's template file for creating new projects 
# using gtsam's CMake tools, see <gtsam>/cmake/example_project/CMakeLists.txt.

cmake_minimum_required(VERSION 3.0)
cmake_policy(SET CMP0048 NEW)
project(libmix4sam VERSION 0.1.0 LANGUAGES CXX C)
set(CMAKE_VERBOSE_MAKEFILE ON)

# Enforce c++11 standards
add_compile_options(-std=c++11) # CMake 3.1 and earlier
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###################################################################################
# OPTIONS

# See whether libmix4sam_unstable is available
if(EXISTS "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}_unstable" AND IS_DIRECTORY "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}_unstable")
  set(LIBMIX4SAM_UNSTABLE_AVAILABLE 1)
else()
  set(LIBMIX4SAM_UNSTABLE_AVAILABLE 0)
endif()

option(USE_LOCAL_GTSAM_BUILD            "If enabled, the user can specify a build and source directory of gtsam which will be used instead of the gtsam-system installation" ON)
option(USE_LOCAL_GTSAM_INSTALL          "If enabled, the user can specify a install directory of gtsam which will be used instead of the gtsam-system installation" OFF)
option(LIBMIX4SAM_BUILD_STATIC_LIBRARY  "whether build static library" OFF)
option(LIBMIX4SAM_BUILD_WRAP            "Enable/Disable building of matlab wrap utility (necessary for matlab interface)" ON)
option(LIBMIX4SAM_BUILD_EXAMPLES        "Enable/Disable building of the example applications" ON)
if(LIBMIX4SAM_UNSTABLE_AVAILABLE)
  option(LIBMIX4SAM_BUILD_UNSTABLE      "Enable/Disable libmix4sam_unstable" ON)
endif()

# Find GTSAM components eighter locally or system-wide install.
if(USE_LOCAL_GTSAM_INSTALL AND USE_LOCAL_GTSAM_BUILD)
  message(FATAL_ERROR "Eigther use local build or local install!")
endif()
if(USE_LOCAL_GTSAM_INSTALL)
  unset(GTSAM_BUILD_DIR CACHE)
  unset(GTSAM_SRC_DIR CACHE)
  set(GTSAM_INSTALL_DIR "$ENV{HOME}/workspace/gtsam/gtsam-42a9-install" CACHE PATH "Install Directory")

  find_package(GTSAM REQUIRED NO_MODULE QUIET
    PATHS "${GTSAM_INSTALL_DIR}"
    NO_DEFAULT_PATH
  )
endif()
if(USE_LOCAL_GTSAM_BUILD)
  unset(GTSAM_INSTALL_DIR CACHE)
  set(GTSAM_BUILD_DIR "$ENV{HOME}/workspace/gtsam/gtsam-42a9-build" CACHE PATH "Build Directory")
  set(GTSAM_INSTALL_DIR "$ENV{HOME}/workspace/gtsam/gtsam-42a9-install" CACHE PATH "Install Directory")
  # Even if we use the local build directory, some cmake files with their dependencies are only
  # available within the install directory.

  # These should be set automatically depending on Build dir by find_package!
  unset(GTSAM_INCLUDE_DIR CACHE)
  unset(GTSAM_DIR CACHE)
  unset(GTSAM_SRC_DIR CACHE)

  find_package(GTSAM REQUIRED NO_MODULE QUIET
    PATHS "${GTSAM_BUILD_DIR}"
    NO_DEFAULT_PATH
  )

  # For using the cmake-scripts
  # (GTSAM_INCLUDE_DIR is a list, we need the first entry)
  LIST(GET GTSAM_INCLUDE_DIR 0 GTSAM_SRC_DIR)

  # We don't need to define include_directories for GTSAM. This is already handled internally.
endif()

if(NOT USE_LOCAL_GTSAM_INSTALL AND NOT USE_LOCAL_GTSAM_BUILD)
  unset(GTSAM_BUILD_DIR CACHE)
  unset(GTSAM_INCLUDE_DIR CACHE)
  unset(GTSAM_SRC_DIR CACHE)
  find_package(GTSAM REQUIRED NO_MODULE)
  find_package(Eigen3 REQUIRED)  # TODO: Don't know if it's needed.
endif()

###################################################################################
# Include GTSAM CMake tools
find_package(GTSAMCMakeTools NO_MODULE QUIET
  PATHS "${GTSAM_INSTALL_DIR}/lib/cmake/GTSAMCMakeTools"
  PATHS "${GTSAM_SRC_DIR}/cmake"
  NO_DEFAULT_PATH
)
if (NOT GTSAMCMakeTools_FOUND)
  find_package(GTSAMCMakeTools)
endif()

include(GtsamMakeConfigFile)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamTesting)    # Easy functions for creating unit tests and scripts
include(GtsamPrinting)   # Additional macros for printing configuration options

# Helper (copied from gtsam 4.1, because it is not present in previous versions)
function(print_config config msg)
  string_pad(padded_config ${GTSAM_PRINT_SUMMARY_PADDING_LENGTH} " ${config}")
  message(STATUS "${padded_config}: ${msg}")
endfunction()

###################################################################################
# BOOST

set(BOOST_FIND_MINIMUM_VERSION 1.65)
set(BOOST_FIND_MINIMUM_COMPONENTS filesystem system thread serialization)

# Following is only necessary if we need a special boost version:
#option(Boost_NO_SYSTEM_PATHS "Enable/Disable Boost system path" ON)
#set(BOOST_ROOT "$ENV{HOME}/workspace/boost/install_1_55_0" CACHE PATH "")
#set(Boost_ADDITIONAL_VERSIONS "1.55" "1.55.0")
#set(BOOST_INCLUDEDIR $ENV{HOME}/boost/install_1_55_0/include)
#set(BOOST_LIBRARYDIR $ENV{HOME}/boost/install_1_55_0/lib)
#set(BOOST_FIND_MINIMUM_VERSION 1.55)

# Boost - same requirement as gtsam
find_package(Boost ${BOOST_FIND_MINIMUM_VERSION} COMPONENTS ${BOOST_FIND_MINIMUM_COMPONENTS} REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR}) # Use SYSTEM to include it via -isystem to deactivate boost's concept_check
###################################################################################

# Ensure that the local folder is searched before the library folders.
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

if(GTSAM_MEX_BUILD_STATIC_MODULE AND LIBMIX4SAM_BUILD_WRAP)
  message(FATAL_ERROR "matlab toolbox needs shared gtsam and libmix4sam lib and can not be build as static wrapper.")
endif()


###################################################################################
# Build static library from common sources
set(CONVENIENCE_LIB_NAME ${PROJECT_NAME})
#target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)
add_subdirectory(${CONVENIENCE_LIB_NAME})

###################################################################################
# Build tests (CMake tracks the dependency to link with GTSAM through our project's static library)
#gtsamAddTestsGlob("example" "tests/test*.cpp" "" "${CONVENIENCE_LIB_NAME}")

###################################################################################
# Build scripts (CMake tracks the dependency to link with GTSAM through our project's static library)
#gtsamAddExamplesGlob("*.cpp" "" "${CONVENIENCE_LIB_NAME}")
# Build examples
if (LIBMIX4SAM_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif(LIBMIX4SAM_BUILD_EXAMPLES)

###################################################################################
# Build libisf_unstable
if (LIBMIX4SAM_BUILD_UNSTABLE)
    add_subdirectory("${CONVENIENCE_LIB_NAME}_unstable")
endif()

###################################################################################
# For gtsam debugging functionalities (debug.h)
OPTION(DEFINE_GTSAM_ENABLE_DEBUG "Activate Custom Debug Messages" ON) # Enabled by default
IF(DEFINE_GTSAM_ENABLE_DEBUG)
    ADD_DEFINITIONS(-DGTSAM_ENABLE_DEBUG)
ENDIF(DEFINE_GTSAM_ENABLE_DEBUG)

###############################################################################
# Matlab Wrapper
if (LIBMIX4SAM_BUILD_WRAP)
  # Build Matlab package
  add_subdirectory(matlab)
endif(LIBMIX4SAM_BUILD_WRAP)

###############################################################################
# Documentation
find_package(Doxygen)
if (DOXYGEN_FOUND)
  add_subdirectory(doc)
endif()

###############################################################################
# Print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
print_config("CMAKE_CXX_COMPILER_ID type" "${CMAKE_CXX_COMPILER_ID}")
print_config("CMAKE_CXX_COMPILER_VERSION" "${CMAKE_CXX_COMPILER_VERSION}")
print_config("CMake version"    "${CMAKE_VERSION}")
print_config("CMake generator"  "${CMAKE_GENERATOR}")
print_config("CMake build tool" "${CMAKE_BUILD_TOOL}")
print_config("Build type" "${CMAKE_BUILD_TYPE}")
message(STATUS "================  Directories  ================================")
print_config("Used CMakeTools Directory" "${GTSAMCMakeTools_DIR}")
if(USE_LOCAL_GTSAM_BUILD)
  print_config("GTSAM-Build-Directory" "${GTSAM_BUILD_DIR}")
  print_config("GTSAM-Include-Dir" "${GTSAM_INCLUDE_DIR}")
endif()
if(USE_LOCAL_GTSAM_INSTALL)
  print_config("GTSAM-Include-Dir" "${GTSAM_INCLUDE_DIR}")
endif()
if(LIBMIX4SAM_UNSTABLE_AVAILABLE)
    print_config("Build libmix4sam_unstable" "${LIBMIX4SAM_BUILD_UNSTABLE}")
endif()
message(STATUS "===============================================================")

# Add all targets to the build-tree export set
export(TARGETS libmix4sam FILE "${PROJECT_BINARY_DIR}/libmix4sam-exports.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE libmix4sam)

set(DEF_INSTALL_CMAKE_DIR lib/cmake/libmix4sam)
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")

file(RELATIVE_PATH CONF_REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/include")

# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(libmix4samConfig.cmake.in
  "${PROJECT_BINARY_DIR}/libmix4samConfig.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "\${${CMAKE_PROJECT_NAME}_CMAKE_DIR}/${CONF_REL_INCLUDE_DIR}")
configure_file(libmix4samConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libmix4samConfig.cmake" @ONLY)

install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libmix4samConfig.cmake"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR}" COMPONENT dev)
 install(EXPORT libmix4sam-exports DESTINATION ${DEF_INSTALL_CMAKE_DIR})

# remove GTSAMCMakeTools_DIR from cmake_gui since it is recomputed anyway
unset(GTSAMCMakeTools_DIR CACHE)
