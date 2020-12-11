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

# Install headers
file(GLOB libmix4sam_headers "*.h")
install(FILES ${libmix4sam_headers} DESTINATION include/${PROJECT_NAME})

set(libmix4sam_srcs "")

# We split the library into separate subfolders, each containing
# tests, timing, and an optional convenience library.
# The following variable is the master list of subdirs to add
set(libmix4sam_subdirs 
  robust
  registration
)

# Optional components
if(EXISTS "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/dataset" AND IS_DIRECTORY "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/dataset")
  list(APPEND libmix4sam_subdirs dataset) 
endif()
if(EXISTS "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/robots" AND IS_DIRECTORY "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/robots")
  list(APPEND libmix4sam_subdirs robots) 
endif()
 
# files want to be excluded
set(excluded_sources "") 
 
# Library sources
foreach(subdir ${libmix4sam_subdirs})
    file(GLOB_RECURSE subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h")
    list(REMOVE_ITEM subdir_srcs "${excluded_sources}")

    if(EXISTS "${subdir}/tests")
        file(GLOB subdir_test_files "${subdir}/tests/*")
        list(REMOVE_ITEM subdir_srcs ${subdir_test_files}) # Remove test files from sources compiled into library
    endif()
  
    list(APPEND libmix4sam_srcs ${subdir_srcs})
    message(STATUS "Building Module: ${subdir}")

    # local and tests
    add_subdirectory(${subdir})
endforeach(subdir)


message(STATUS "Building libmix4sam - shared")
#message(STATUS "lasve-DEBUG libmix4sam_srcs: ${libmix4sam_srcs}")
add_library(${CONVENIENCE_LIB_NAME} SHARED ${libmix4sam_srcs})
target_link_libraries(${CONVENIENCE_LIB_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_REGEX_LIBRARY} gtsam)

set_target_properties(${CONVENIENCE_LIB_NAME} PROPERTIES
        OUTPUT_NAME         libmix4sam
        CLEAN_DIRECT_OUTPUT 1
)

install(TARGETS libmix4sam EXPORT libmix4sam-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin PUBLIC_HEADER DESTINATION include/${PROJECT_NAME})
#list(APPEND GTSAM_EXPORTED_TARGETS gtsam)

###################################################################################
# Build MATLAB wrapper (CMake tracks the dependecy to link with GTSAM through our project's static library)
if (LIBMIX4SAM_BUILD_WRAP)
  include(GtsamMatlabWrap)   # Set up automatic MATLAB wrapper generation

  # Generate, build and install toolbox
  set(mexFlags "${GTSAM_BUILD_MEX_BINARY_FLAGS}")
  if(NOT BUILD_SHARED_LIBS)
          list(APPEND mexFlags -DGTSAM_IMPORT_STATIC)
  endif()

  # Wrap
  wrap_and_install_library(../libmix4sam.h "${CONVENIENCE_LIB_NAME};gtsam" "${CMAKE_CURRENT_SOURCE_DIR}" "" "${mexFlags}")

  # install matlab functions and scripts
  #add_subdirectory(matlab)
endif(LIBMIX4SAM_BUILD_WRAP)







