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


#include "element.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

#include "craftui_fileutils.h"
#include "craftui_mathutils.h"


void Element::loadFromFileStorage(const cv::FileNode& node) {
    node["typename"] >> elementTypeName;
    node["id"] >> id;

    /* collision configuration */
    node["numPointsThresh"] >> numPointsThresh;
    node["maxDistance"] >> maxDistance;

    /* load the hull */
    hullCloud = loadCloudFromFileNode<pcl::PointXYZRGBA>( node["hullCloud"] );   
    setupHullFilter();

    /* load the plane model */
    cv::FileNode pnode = node["plane"];
    if (pnode.type() == cv::FileNode::SEQ) {
        auto it=pnode.begin();
        plane[0] = *it++;
        plane[1] = *it++;
        plane[2] = *it++;
        plane[3] = *it;
    }
}


void Element::saveToFileStorage(cv::FileStorage& fs) const {
    fs << "typename" << elementTypeName;
    fs << "id" << id;

    /* collision configuration */
    fs << "numPointsThresh" << numPointsThresh;
    fs << "maxDistance" << maxDistance;

    fs << "hullCloud";
    safeCloudToStorage<pcl::PointXYZRGBA>(*hullCloud, fs);

    /* write the plane model */
    fs << "plane" << "[";
        fs << plane[0];
        fs << plane[1];
        fs << plane[2];
        fs << plane[3];
    fs << "]";
}


void Element::defineFromCloud(const Cloud::Ptr& cloud) {
    computeHull(cloud);
}


void Element::computeHull(const Cloud::ConstPtr& cloud) {

    /* fit a plane to the cloud */
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (200);
    seg.setDistanceThreshold (0.02);

    seg.setInputCloud(cloud);

    pcl::PointIndices::Ptr planeindices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients
        (new pcl::ModelCoefficients);

    seg.segment(*planeindices, *coefficients);

    //TODO check if the plane is somehow parallel to the calib plane

    /* project all points on the plane */
    Cloud::Ptr planeCloud(new Cloud);
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*planeCloud);
    
    /* compute the concave hull */
    Cloud::Ptr hull(new Cloud);
    pcl::ConcaveHull<PointT> cHull;
    cHull.setInputCloud(planeCloud);
    cHull.setAlpha(1);
    cHull.setDimension(2);
    cHull.reconstruct(*hull);

    /* store the concave hull */
    hullCloud = hull;
    planeCoefficients = coefficients;
    plane = planeCoefficientsToParameters(*coefficients);

    /* setup collision filter */
    setupHullFilter();
}


void Element::setupHullFilter() {
    /* dummy vertices */
    std::vector<pcl::Vertices> vertices;
    pcl::Vertices vt;
    for (int i=0; i<hullCloud->size(); i++) {
        vt.vertices.push_back(i);
    }
    vertices.push_back(vt);

    /* configute the cropHull filter */
    cropHull.setHullCloud(hullCloud);
    cropHull.setHullIndices(vertices);
    cropHull.setDim(2);
}


bool Element::collidePoint(const pcl::PointXYZRGBA& p) {

    /* check if the point is within distance */
    Eigen::Vector4f pv(p.x, p.y, p.z, 1);
    float distance = pv.dot(plane);
    if (distance > maxDistance) {
        return false;
    }

    /* project the point to the plane */
    pcl::PointXYZRGBA pp;
    pp.x = p.x - distance * plane[0];
    pp.y = p.y - distance * plane[1];
    pp.z = p.z - distance * plane[2];

    /* check if the point is within the concave hull */
    //TODO
    

    return false;
}


pcl::PointIndices::Ptr Element::collideCloud(const Cloud::ConstPtr cloud) {

    pcl::PointIndices::Ptr baseIndices (new pcl::PointIndices ());
    pcl::PointIndices::Ptr planeIndices (new pcl::PointIndices ());

    if (isTriggered()) return baseIndices;

    /* first remove NaNs */
    pcl::removeNaNFromPointCloud(*cloud, baseIndices->indices);

    /* check if the points are within distance */
    Eigen::Hyperplane<float, 3> hPlane;
    hPlane.coeffs() = plane;
    for (int i : baseIndices->indices) {
        PointT p = cloud->at(i);
        Eigen::Vector3f pv(p.x, p.y, p.z);
        float distance = hPlane.absDistance(pv);
        if (std::abs(distance) < maxDistance) {
            planeIndices->indices.push_back(i);
        }
    }

    /* not enough collisions possible */
    if (planeIndices->indices.size() < numPointsThresh) return planeIndices;

    /* get all points within the concave hull */
    pcl::PointIndices::Ptr collIndices(new pcl::PointIndices);
    cropHull.setInputCloud(cloud);
    cropHull.setIndices(planeIndices);
    cropHull.filter(collIndices->indices);

    numCollisions += collIndices->indices.size();
    return collIndices;
}


bool Element::isTriggered() const {
    return (numCollisions >= numPointsThresh + dynamicThresh);
}


void Element::resetCollision() {
    numCollisions = 0;
}


int Element::getNumCollisions() const {
    return numCollisions;
}

