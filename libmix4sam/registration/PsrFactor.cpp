/***************************************************************************
 * libmix4sam - Mixtures for Smoothing and Mapping Library
 *
 * Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
 * For more information see https://mytuc.org/mix
 *
 * libmix4sam is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libmix4sam is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contact Information: Sven Lange (sven.lange@etit.tu-chemnitz.de)
 ***************************************************************************/

/**
 * @file PsrFactor.cpp
 * @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)
 */

#include <libmix4sam/registration/PsrFactor.h>
//#include "gtsam_unstable/slam/TransformBtwRobotsUnaryFactorEM.h"

// for using the matrix logarithm:
//#include <unsupported/Eigen/MatrixFunctions>

#define USE_SIMPLE_D2D 0

namespace libmix4sam {

// void Psr2DPriorFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const {
//   Base::print(s);
// }

// bool Psr2DPriorFactor::equals(const NonlinearFactor &expected, double tol) const {
//   const This *e = dynamic_cast<const This *>(&expected);
//   return e != NULL && Base::equals(*e, tol);
// }

// void PsrRadarPriorFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const {
//   Base::print(s);
// }

// bool PsrRadarPriorFactor::equals(const NonlinearFactor &expected, double tol) const {
//   const This *e = dynamic_cast<const This *>(&expected);
//   return e != NULL && Base::equals(*e, tol);
// }

}