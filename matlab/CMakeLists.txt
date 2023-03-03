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

# Info: GTSAM's wrapping functionality is already configured and included in the main CMakeLists.txt

# Set up cache options
option(WRAP_MEX_BUILD_STATIC_MODULE "Build MATLAB wrapper statically (increases build time)" OFF)
set(WRAP_BUILD_MEX_BINARY_FLAGS     "" CACHE STRING "Extra flags for running Matlab MEX compilation")
set(WRAP_TOOLBOX_INSTALL_PATH       "${CMAKE_INSTALL_PREFIX}/libmix4sam_toolbox"
    CACHE
      PATH
      "Matlab toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/libmix4sam_toolbox")
option(WRAP_BUILD_TYPE_POSTFIXES    "Enable/Disable appending the build type to the name of compiled libraries" ON)

# Copy matlab.h to the correct folder.
configure_file(${GTSAM_INSTALL_DIR}/include/gtwrap/matlab.h
  ${PROJECT_BINARY_DIR}/wrap/matlab.h COPYONLY)

# Include the Matlab-related code.
include(MatlabWrap)

# Ensure that the local folder is searched before the library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

# Install library
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# ##############################################################################
# Build MATLAB wrapper
# (CMake tracks the dependency to link with GTSAM through our project's static library)
set(interface_files "${PROJECT_SOURCE_DIR}/libmix4sam.i")
matlab_wrap("${interface_files}" "${PROJECT_NAME}" "" "" "" "")

# Wrap version for libmix4sam_unstable
if(LIBMIX4SAM_BUILD_UNSTABLE)
  set(interface_files "${PROJECT_SOURCE_DIR}/${CONVENIENCE_LIB_NAME}_unstable/libmix4sam_unstable.i")
  matlab_wrap("${interface_files}"                           # interface Header
              "${CONVENIENCE_LIB_NAME}_unstable"             # Module name
              ""                                             # Additional Libraries to link
              ""                                             # Extra Include Dirs
              "${mexFlags}"                                  # Extra mex flags
              "")                                            # Ignore Classes
endif(LIBMIX4SAM_BUILD_UNSTABLE)

# Add the Matlab scripts that are a part of the toolbox
install_matlab_scripts("${PROJECT_SOURCE_DIR}/matlab/" "*.m;*.fig")

# We print out our configuration for an easy visual check
message(STATUS "MATLAB Root: ${MATLAB_ROOT}")
message(STATUS "Mex Compiler: ${Matlab_MEX_COMPILER}")
