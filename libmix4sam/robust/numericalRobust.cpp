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

#include "libmix4sam/robust/numericalRobust.h"

using namespace gtsam;

namespace libmix4sam
{
  double ScaledLogSumExp(Vector Exponents, Vector Scaling)
  {
    double Sum = 0;

    double MaxExp = Exponents.maxCoeff();

    for (unsigned int i = 0; i < Exponents.size(); i++)
    {
      Sum += exp(Exponents(i) - MaxExp) * Scaling(i);
    }
    return log(Sum) + MaxExp;
  }

  double ScaledDivSumExp(double UpperExponent, Vector LowerExponents, Vector LowerScaling)
  {
    double Sum = 0;

    double MaxExp = LowerExponents.maxCoeff();

    for (unsigned int i = 0; i < LowerExponents.size(); i++)
    {
      Sum += exp(LowerExponents(i) - MaxExp) * LowerScaling(i);
    }
    return exp(UpperExponent - MaxExp) / Sum;
  }
}

