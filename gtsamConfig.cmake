# Set the build type for convenience
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "")

# We do not need the unstable parts of gtsam
set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "")

# For using the gtsam matlab wrapper in combination with matlab's
# parallel processing toolbox, we need to deactivate gtsam's TBB option!
set(GTSAM_WITH_TBB OFF CACHE BOOL "")

# We use the matrix logarithm, which is in eigen's unsupported branch
set(GTSAM_WITH_EIGEN_UNSUPPORTED ON CACHE BOOL "")

# We want to use the matlab wrapper
set(GTSAM_INSTALL_MATLAB_TOOLBOX ON CACHE BOOL "")
set(GTSAM_UNSTABLE_INSTALL_MATLAB_TOOLBOX OFF CACHE BOOL "")

# Enable compatibility to 4.2 -> TODO, do we need this?
set(GTSAM_ALLOW_DEPRECATED_SINCE_V42 ON CACHE BOOL "")

# We don't need the examples
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "")

# Hopefully we can spare time by not building the tests
set(GTSAM_BUILD_TESTS OFF CACHE BOOL "")

# We don't need timing
set(GTSAM_BUILD_TIMING_ALWAYS OFF CACHE BOOL "")

# Disable the python wrapper, if not needed.
set(GTSAM_BUILD_PYTHON ON CACHE BOOL "")
set(GTSAM_UNSTABLE_BUILD_PYTHON OFF CACHE BOOL "")

# If we are working within the conda environment, we need to give gtsam 
# some hints, where to find the correct python environment
if(DEFINED ENV{CONDA_PREFIX})
  set(Python_FIND_STRATEGY LOCATION CACHE INTERNAL "")
  set(Python3_FIND_STRATEGY LOCATION CACHE INTERNAL "")
  set(PYTHON_ROOT_DIR "$ENV{CONDA_PREFIX}" CACHE PATH "")
  set(Python_ROOT_DIR "$ENV{CONDA_PREFIX}" CACHE PATH "")
  set(Python3_ROOT_DIR "$ENV{CONDA_PREFIX}" CACHE PATH "")
endif()