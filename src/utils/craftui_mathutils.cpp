/*
   * (C) 2015, Aurel Wildfellner
   *
   * This file is part of CraftUI.
   *
   * Beholder is free software: you can redistribute it and/or modify
   * it under the terms of the GNU General Public License as published by
   * the Free Software Foundation, either version 3 of the License, or
   * (at your option) any later version.
   *
   * Beholder is distributed in the hope that it will be useful,
   * but WITHOUT ANY WARRANTY; without even the implied warranty of
   * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   * GNU General Public License for more details.
   *
   * You should have received a copy of the GNU General Public License
   * along with CraftUI. If not, see <http://www.gnu.org/licenses/>. */


#include "craftui_mathutils.h"


Eigen::Vector4f planeCoefficientsToParameters(const pcl::ModelCoefficients& coef) {
    Eigen::Vector4f vec;
    vec[0] = coef.values[0];
    vec[1] = coef.values[1];
    vec[2] = coef.values[2];
    vec[3] = coef.values[3];
    return vec;
}

