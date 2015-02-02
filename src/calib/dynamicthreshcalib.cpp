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

#include "dynamicthreshcalib.h"


DynamicThreshCalib::DynamicThreshCalib(const std::vector<Element::Ptr>& elements) {
    uiElements = elements;
}


void DynamicThreshCalib::calibWithCloud(const Cloud::ConstPtr& cloud) {
    
    for (auto element : uiElements) {
        /* get number of collisions and update dynamicThresh */
        element->resetCollision();
        element->collideCloud(cloud);
        int numCollisions = element->getNumCollisions();
        if (numCollisions > element->dynamicThresh) {
            element->dynamicThresh = numCollisions;
        }
    }

}

