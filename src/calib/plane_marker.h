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

#include <iostream>
#include <cmath>

#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp> 

#include <pcl/common/common_headers.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>


#ifndef __PLANE_MARKER_H__
#define __PLANE_MARKER_H__

//#define DEBUG_MARKER
#ifdef DEBUG_MARKER
    #define DEBUG(s) printf("[PlaneMarker] "); printf(s); printf("\n")
    #define DEBUG_P(s,p) printf("[PlaneMarker] " ); printf(s,p); printf("\n")
#else
    #define DEBUG(s)
    #define DEBUG_P(s,p)
#endif


template<typename PointT> 
class PlaneMarker {

    public:

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

    public:

        PlaneMarker (float length=0.4, float width=0.2, float sigma=0.03);

        void setMarkerDimensions(float length, float width, float sigma=0.03);

        bool computeMarkerCenter(PointCloudConstPtr cloud, PointT& center);

        /* some points describing the marker */
        PointT pointA, pointB, centerPoint;
        PointT pointC, pointD;
        PointT pointR;

        /* the cloud of the marker */
        PointCloudConstPtr markerCloud;
       
    private:

        /* marker properties */ 
        float marker_length, marker_width, marker_diagonal;
        float marker_sigma;

        /* search properties */
        int maxSACIterations;
        float sacDistanceThresh;
        float clusterTolerance;
        int minClusterSize;

        /* classification methods */
        bool checkCandidate(const PointCloudPtr pc_ptr);
        bool findMaxDistandPoint(const PointCloudPtr pc_ptr, 
                const float limit,
                const PointT& start_point, PointT& max_point);
        bool mainDiagonal(const PointCloudPtr pc_ptr, const float limit);
        bool secondDiagonal(const PointCloudPtr pc_ptr, const float limit);
        void centerBetweenPoints(const PointT& a, 
                const PointT& b, PointT& c);
        void centerFromMainDiagonal();
        void centerFromBothDiagonals();

};


template <typename PointT> PlaneMarker<PointT>::PlaneMarker 
    (float length, float width, float sigma) :
    pointA(),
    pointB()
{
    setMarkerDimensions(length, width, sigma);

    /* search properties */
    minClusterSize = 2000;
    clusterTolerance = 0.05; //5cm
    maxSACIterations = 1000;
    sacDistanceThresh = 0.06;
}


template <typename PointT> void PlaneMarker<PointT>::setMarkerDimensions(
        float length, float width, float sigma)
{
    /* length should be greater then width */
    if (width > length) {
        marker_length = width;
        marker_width = length;
    } else {
        marker_length = length;
        marker_width = width;
    }
    marker_diagonal = std::sqrt(length*length + width*width);

    /* the valid delta for the found diagonal */
    marker_sigma = sigma;
}


template <typename PointT> bool PlaneMarker<PointT>::computeMarkerCenter(
        PointCloudConstPtr inCloud, 
        PointT& center) 
{
    /* do some preprocessing to the cloud */
    PointCloudPtr fcloud (new PointCloud);

    /* remove NaNs */
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*inCloud, *fcloud, mapping);

/*          
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(fcloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*fcloud);

    // takes long, messes up results
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (fcloud);
    vg.setLeafSize (0.005f, 0.005f, 0.005f);
    vg.filter (*fcloud);
*/

    /* segment all euclidean clusters in the cloud */
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree 
        (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud (fcloud);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(fcloud);
    ec.extract(cluster_indices);

    /* search for markers in individual clusters */
    for (auto clusteri : cluster_indices) {

        /* if the cluster is too small, discrard */
        if (clusteri.indices.size() < minClusterSize) {
            continue;
        }

        /* get a shared pointer for the cluster indices */
        pcl::PointIndices::Ptr clusteri_ptr(new pcl::PointIndices);
        clusteri_ptr->indices = clusteri.indices;

        /* extract the largest plane from the cluster */
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (maxSACIterations);
        seg.setDistanceThreshold (sacDistanceThresh);

        seg.setInputCloud(fcloud);
        seg.setIndices(clusteri_ptr);

        pcl::PointIndices::Ptr planeindices(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients
            (new pcl::ModelCoefficients);
        seg.segment(*planeindices, *coefficients);

        /* extract the plane cloud */
        PointCloudPtr planeCloud(new PointCloud); 
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(fcloud);
        extract.setIndices(planeindices);
        extract.filter(*planeCloud);

        /* project it on the plane */
        pcl::ProjectInliers<PointT> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(planeCloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*planeCloud);

        /* check the plane for the marker */
        bool foundMarker = checkCandidate(planeCloud); 

        if (foundMarker) {
            /* the marker center*/
            centerFromBothDiagonals();
            center = centerPoint;
            markerCloud = planeCloud;
            return true;
        }
    }

    /* no plane found */
    return false;
}


template <typename PointT> bool PlaneMarker<PointT>::checkCandidate(
        const PointCloudPtr pc_ptr)
{

    if (!mainDiagonal(pc_ptr, marker_diagonal+marker_sigma)) {
        DEBUG("early exit, cluster too big");
        return false;
    }

    if (!(std::abs( pcl::euclideanDistance(pointA, pointB) - marker_diagonal)               <= marker_sigma)) 
    {
        return false;
    }

    if (!secondDiagonal(pc_ptr, marker_diagonal+marker_sigma)) {
        DEBUG("no second diagonal");
        return false;
    }

    float short_side = (marker_length > marker_width) ? 
        marker_width : marker_length;

    // check marker dimensions. length > width
    if (pcl::euclideanDistance(pointA, pointD) < (marker_width - marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointA, pointC) < (marker_width - marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointB, pointD) < (marker_width - marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointB, pointC) < (marker_width - marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointA, pointD) >= (marker_length + marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointA, pointC) >= (marker_length + marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointB, pointD) >= (marker_length + marker_sigma)) 
        return false;
    if (pcl::euclideanDistance(pointB, pointC) >= (marker_length + marker_sigma)) 
        return false;
    return true;
}


template <typename PointT> bool PlaneMarker<PointT>::findMaxDistandPoint(
        const PointCloudPtr pc_ptr, 
        const float limit,
        const PointT& start_point, PointT& max_point) 
{

    // find the first corner
    float max_distance = 0.0f;
    for (PointT point : pc_ptr->points) {
        float c_distance = 
            pcl::euclideanDistance(point, start_point);
        if (c_distance > max_distance)  {
            max_distance = c_distance;
            max_point = point;
            // early exit, if cluster is too big
            if (max_distance > limit) {
                DEBUG_P("max distance: %f", max_distance);
                return false;
            }
        }
    }

    DEBUG_P("max distance: %f", max_distance);
    return true;
}


template <typename PointT> bool PlaneMarker<PointT>::mainDiagonal(
        const PointCloudPtr pc_ptr, 
        const float limit) 
{
    // start with a 'random' point
    PointT point_r = pc_ptr->points[0];
    pointR = point_r;

    if (! findMaxDistandPoint(pc_ptr, limit, point_r, pointA)) 
        return false;
    if (! findMaxDistandPoint(pc_ptr, limit, pointA, pointB)) 
        return false; 

    return true;
} 


template <typename PointT> bool PlaneMarker<PointT>::secondDiagonal(
        const PointCloudPtr pc_ptr, 
        const float limit) 
{
    PointT point_r;

    float distCenter_thres = marker_diagonal / 3.0;
    float distAB_thresh = distCenter_thres / 2.0;

    /* search for a candidate point to start */
    bool found_candidate = false;
    for (PointT point : pc_ptr->points) {
        if (pcl::euclideanDistance(point, centerPoint) 
                > distCenter_thres) 
        {
            float distA = pcl::euclideanDistance(point, pointA);
            float distB = pcl::euclideanDistance(point, pointB);
            if (std::abs(distA - distB) < distAB_thresh) {
                found_candidate = true;
                point_r =  point;
                break;
            }
        } 
    }

    // no candidate found, not a valid marker
    if (!found_candidate) {
        return false;
    }

    // not likely to fail on limit
    findMaxDistandPoint(pc_ptr, limit, point_r, pointC);
    findMaxDistandPoint(pc_ptr, limit, pointC, pointD);

    return true;
}


template <typename PointT> void PlaneMarker<PointT>::centerBetweenPoints(
        const PointT& a, 
        const PointT& b, PointT& c) 
{
    Eigen::Vector3f pA(a.x, a.y, a.z);
    Eigen::Vector3f pB(b.x, b.y, b.z);
    Eigen::Vector3f middle = pA + ((pB - pA) /2.0);
    c.getVector3fMap() = middle;
}


template <typename PointT> void PlaneMarker<PointT>::centerFromMainDiagonal() {
    centerBetweenPoints(pointA, pointB, centerPoint);
}


template <typename PointT> void PlaneMarker<PointT>::centerFromBothDiagonals() {
    PointT p0, p1;
    centerBetweenPoints(pointA, pointB, p0);
    centerBetweenPoints(pointC, pointD, p1);
    centerBetweenPoints(p0, p1, centerPoint);
}


#endif

