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

# Build doxygen documentation as configuration option
option(BUILD_DOC                  "Enable/Disable building of doxygen docs"   ON)

# add a target to generate API documentation with Doxygen
if (BUILD_DOC)

    set(libmix4sam_doc_subdirs 
        ${PROJECT_NAME}/robust
        ${PROJECT_NAME}/registration
        ${PROJECT_NAME}
    )

    # Optional components
    if(EXISTS "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/dataset" AND IS_DIRECTORY "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/dataset")
        list(APPEND libmix4sam_doc_subdirs ${PROJECT_NAME}/dataset) 
    endif()
    if(EXISTS "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/robots" AND IS_DIRECTORY "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/robots")
        list(APPEND libmix4sam_doc_subdirs ${PROJECT_NAME}/robots) 
    endif()
    
    # Optional UNSTABLE components
    set(libmix4sam_unstable_doc_subdirs
        ${PROJECT_NAME}_unstable/gnss
        ${PROJECT_NAME}_unstable
    )
    
    # Build a list of folders to include depending on build options
    set(doc_subdirs ${libmix4sam_doc_subdirs})
    if (LIBMIX4SAM_BUILD_UNSTABLE)
        list(APPEND doc_subdirs ${libmix4sam_unstable_doc_subdirs})
    endif()
    if (LIBMIX4SAM_BUILD_EXAMPLES)
        list(APPEND doc_subdirs examples)
    endif()
    
    # From subfolders, build a list with whitespace separation of paths
    set(LIBMIX4SAM_DOXYGEN_INPUT_PATHS "")
    foreach(dir ${doc_subdirs})
        set(LIBMIX4SAM_DOXYGEN_INPUT_PATHS "${LIBMIX4SAM_DOXYGEN_INPUT_PATHS} ${PROJECT_SOURCE_DIR}/${dir}")
    endforeach()

    # Generate and configure Doxyfile
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    
    # Add target to actually build documentation as configured
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
    
endif()