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


#ifndef __DYNAMIC_THRESH_CALIB_H__
#define __DYNAMIC_THRESH_CALIB_H__

#include <boost/bind.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>

#include "elementstorage.h"
#include "colordescriptor.h"
#include "elementtype.h"
#include "element.h"

class DynamicThreshCalib {

    public:

        typedef typename pcl::PointXYZRGBA PointT; 
        typedef typename pcl::PointCloud<PointT> Cloud;

    public:

        DynamicThreshCalib(const std::vector<Element::Ptr>& elements);

        void calibWithCloud(const Cloud::ConstPtr& cloud);

    private:

        std::vector<Element::Ptr> uiElements;

};


#endif

