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


#ifndef __ELEMENT_H__
#define __ELEMENT_H__

#include <iostream>
#include <string>
#include <memory>

#include <pcl/common/common_headers.h>
#include <pcl/filters/crop_hull.h>

#include <opencv2/core/core.hpp>

#include "elementvisitor.h"


class Element {

    public:

        typedef typename std::shared_ptr<Element> Ptr;

        typedef typename pcl::PointXYZRGBA PointT;
        typedef typename pcl::PointCloud<PointT> Cloud;

        /* visitor */
        virtual void accept(ElementVisitor& visitor) = 0;

        /* file storage */
        virtual void loadFromFileStorage(const cv::FileNode&);
        virtual void saveToFileStorage(cv::FileStorage&) const;

        /* naming */
        std::string elementTypeName = "element";
        std::string id = "unnamed";

        /* collison parameters */
        int numPointsThresh;
        int dynamicThresh = 0;
        float maxDistance;

        /*** collision interface ***/
        /* collide a single point with the ui element */
        virtual bool collidePoint(const pcl::PointXYZRGBA& p);
        /* collide an cloud, returns trigger status */
        virtual pcl::PointIndices::Ptr collideCloud(const Cloud::ConstPtr cloud);
        /* the number of collisions have reached a threshhold */
        virtual bool isTriggered() const;
        /* returns the number of collided points since last reset */
        virtual int getNumCollisions() const;
        /* reset all collisions */
        virtual void resetCollision();

        /* calibration */
        virtual void defineFromCloud(const Cloud::Ptr& cloud);

        /* orientation and geometry */
        Cloud::Ptr hullCloud; 
        Eigen::Vector4f plane;

    protected:

        void computeHull(const Cloud::ConstPtr& cloud);
        /* needs to be called after a new hull is computet or loaded from a file */
        void setupHullFilter();
        pcl::CropHull<PointT> cropHull;

        int numCollisions = 0;

        pcl::ModelCoefficients::Ptr planeCoefficients;

};


#endif

