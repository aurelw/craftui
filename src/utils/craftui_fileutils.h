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


#ifndef __CRAFTUI_FILEUTILS_H__
#define __CRAFTUI_FILEUTILS_H__

#include <iostream>
#include <string>
#include <memory>

#include <pcl/common/common_headers.h>
#include <opencv2/core/core.hpp>


/* safes a pointcloud to an OpenCV file storage.
 * Only X, Y, Z are stored, even for XYZRGBA clouds */
template<typename PointT>
void safeCloudToStorage(const pcl::PointCloud<PointT>& cloud, cv::FileStorage& fs) {
    fs << "[";
    for (auto it = cloud.begin(); it != cloud.end(); it++) {
        PointT p = *it;
        fs << p.x << p.y << p.z;
    }
    fs << "]";
}


/* safes the X, Y, Z coordinates of a PointCloud to a OpenCV FileNode */
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr loadCloudFromFileNode(const cv::FileNode& node) {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (node.type() == cv::FileNode::SEQ) {
        for (auto it=node.begin(); it!=node.end();) {
            PointT p;
            p.x = *it; it++;
            p.y = *it; it++;
            p.z = *it; it++;
            cloud->push_back(p);
        }
    }

    return cloud;
}


#endif

