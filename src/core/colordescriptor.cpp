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

#include "colordescriptor.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <pcl/point_types_conversion.h>


void ColorDescriptor::pointToHSV(const PointT& p,
        float& h, float& s, float& v)
{
    pcl::PointXYZHSV phsv;
    pcl::PointXYZRGBA tp = p;
    pcl::PointXYZRGBAtoXYZHSV(tp, phsv);
    h = phsv.h;
    s = phsv.s;
    v = phsv.v;

}


float ColorDescriptor::hueDistance(const float hue0, const float hue1) {

    float distance;
    if (hue0 < hue1) {
        distance = hue1 - hue0;
    } else {
        distance = hue0 - hue1;
    }

    if (distance > 180.0) {
        return distance - 180.0;
    } else {
        return distance;
    }
        
}


void ColorDescriptor::setMinimumSat(const float sat) {
    minimumSat = sat;
}


void ColorDescriptor::compute(const Cloud::ConstPtr& cloud) {

    /* two histograms */
    int hueBins = 40; // hue in 0.0-360.0
    int satBins = 20; // sat 0.0 - 1.0
    std::vector<int> hueHist(hueBins, 0);
    std::vector<int> satHist(satBins, 0);
    
    for (auto it = cloud->begin(); it != cloud->end(); it++) {
        float h, s, v;
        pointToHSV(*it, h, s, v);

        if (s > minimumSat) {
            int hBin = std::round((h / 360) * (hueBins-1));
            int sBin = std::round(s * (satBins-1));
            std::cout << "hBin: " << hBin << " sBin: " << sBin << std::endl;
            hueHist[hBin] += 1;
            satHist[sBin] += 1;
        }
    }

    /* get the peak in the histogram and the according value for the bin */
    primaryHue = std::distance( hueHist.begin(),
            std::max_element(hueHist.begin(), hueHist.end()) ) * (360.0/hueBins);
    primarySat = std::distance( satHist.begin(),
            std::max_element(satHist.begin(), satHist.end()) ) * (1.0/satBins);

}


float ColorDescriptor::getPrimaryHue() const {
    return primaryHue;
}


float ColorDescriptor::getPrimarySat() const {
    return primarySat;
}

