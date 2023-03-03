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
# Info: Inspired by https://github.com/borglab/gtsam-project-python
# Info: Inspired by https://github.com/borglab/gpmp2

# We use this as a convenience variable. It points to `python/` in this case.
#set(LIBMIX4SAM_PYTHON_MODULE_PATH ${PROJECT_BINARY_DIR}/python)
file(TO_NATIVE_PATH "${PROJECT_BINARY_DIR}/python" LIBMIX4SAM_PYTHON_MODULE_PATH)

# Copy over the setup.py.in file. This will become `python/setup.py`.
configure_file(${PROJECT_SOURCE_DIR}/python/templates/setup.py.in
               ${LIBMIX4SAM_PYTHON_MODULE_PATH}/setup.py)

configure_file(${PROJECT_SOURCE_DIR}/python/requirements.txt
               ${LIBMIX4SAM_PYTHON_MODULE_PATH}/requirements.txt COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/python/templates/${PROJECT_NAME}.tpl
               ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl)

# Copy over python files for libmix4sam.
file(COPY ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME}
     DESTINATION ${LIBMIX4SAM_PYTHON_MODULE_PATH})

### Set the name(s) of the interface file(s) used for wrapping
set(interface_headers
  ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}.i
  ${ADDITIONAL_INTERFACE_FILES}
)

pybind_wrap(
  ${PROJECT_NAME}_py # target
  "${interface_headers}" # interface header file
  "${PROJECT_NAME}.cpp" # the generated cpp
  "${PROJECT_NAME}" # module_name
  "${PROJECT_NAME}" # top namespace in the cpp file
  "${ignore}" # ignore classes
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl
  ${PROJECT_NAME} # libraries to link against
  "${PROJECT_NAME}" # dependencies to build first
  ON # use boost
)

set_target_properties(
  ${PROJECT_NAME}_py
  PROPERTIES OUTPUT_NAME "${PROJECT_NAME}"
  LIBRARY_OUTPUT_DIRECTORY "${LIBMIX4SAM_PYTHON_MODULE_PATH}/${PROJECT_NAME}"
  DEBUG_POSTFIX "" # Otherwise you will have a wrong name
  RELWITHDEBINFO_POSTFIX "" # Otherwise you will have a wrong name
)

# Add custom target so we can install with `make python-install`
add_custom_target(
  python-install
  COMMAND ${PYTHON_EXECUTABLE} -m pip install .
  DEPENDS ${PROJECT_NAME}_py
  WORKING_DIRECTORY ${LIBMIX4SAM_PYTHON_MODULE_PATH})