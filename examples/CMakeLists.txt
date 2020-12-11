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


add_executable(testPsr2DFactor testPsr2DFactor.cpp)
target_link_libraries(testPsr2DFactor 
  gtsam
  ${CONVENIENCE_LIB_NAME}
  ${catkin_LIBRARIES}
)


