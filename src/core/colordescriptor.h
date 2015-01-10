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


#ifndef __COLOR_DESCRIPTOR_H__
#define __COLOR_DESCRIPTOR_H__


#include <pcl/common/common_headers.h>

#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>

#include "element.h"


class ColorDescriptor {

    public:

        typedef typename pcl::PointXYZRGBA PointT;
        typedef typename pcl::PointCloud<PointT> Cloud;

    public:

        static void pointToHSV(const PointT& p,
                float& h, float& s, float& v);

    public:

        void compute(const Cloud::ConstPtr& cloud);

        float getPrimaryHue();
        float getPrimarySat();

    protected:

        float primaryHue = 0.0;
        float primarySat = 0.0;

};


#endif

