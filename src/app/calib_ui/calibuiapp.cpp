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


#include "calibuiapp.h"

#include <pcl/filters/plane_clipper3D.h>

#include "craftui_mathutils.h"



void CalibUIApp::run() {

    Cloud::Ptr capturedCloud;
    Cloud::Ptr planeCloud(new Cloud); 
    Cloud::Ptr emptyCloud(new Cloud); 
    bool foundMarker = false;
    
    while (!foundMarker && !doQuit && !viewer.wasStopped()) {
        updateCloud(); 
        viewer.showCloud(cloud);

        if (doCaptureMarker) {
            capturedCloud = cloud;
            /* search for the calibration pattern */
            foundMarker = planeMarker.computeMarkerCenter(capturedCloud, 
                    markerPoint);
            break;
        }

        if (doQuit) {
            return;
        }
    }

    /* the marker is found, handle the plane */
    if (foundMarker) {

        /* learn a ColorDescriptor from the marker */
        if (learnDescriptor) {
            defaultDescriptor.compute(planeMarker.markerCloud);
        }

        /* remove the points belonging to the marker */
        pcl::ExtractIndices<pcl::PointXYZRGBA> extractmarker;
        extractmarker.setInputCloud(capturedCloud);
        extractmarker.setIndices(planeMarker.markerIndices);
        extractmarker.setNegative(true);
        extractmarker.filter(*capturedCloud);

        /* get the indicies for all points on the plane */
        pcl::PointIndices::Ptr planeInliers (new pcl::PointIndices ());
        Eigen::Vector4f plane = planeCoefficientsToParameters(
                *planeMarker.markerCoefficients);
        pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> scModelPlane(capturedCloud);
        scModelPlane.selectWithinDistance(plane, 
                uiPlaneTollerance, planeInliers->indices);
       
        /* extract the indices */
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud(capturedCloud);
        extract.setIndices(planeInliers);
        extract.filter(*planeCloud);

        /* show the cloud */
        viewer.showCloud(planeCloud);

        //TODO remove the marker cloud
        /* extract and define the UI elements from the plane */
        extractElements(planeCloud);
        
    }

    /* clear the main cloud */
    viewer.showCloud(emptyCloud);

    /* display the convex hulls */
    int idx = 0;
    auto elements = elementStorage->getElements();
    for (auto element : elementStorage->getElements()) {
        viewer.showCloud(element->hullCloud, std::to_string(idx++));
    }


    while (!doQuit && !viewer.wasStopped()) {
        updateCloud(); 

        /* do collision tests */
        pcl::PointIndices::Ptr ecIndices;
        for (auto element : elementStorage->getElements()) {
            element->resetCollision();
            pcl::PointIndices::Ptr tmpIndices = element->collideCloud(cloud);
            if (tmpIndices->indices.size() > 50) {
                ecIndices = tmpIndices;
            }
        }

        /* extract the indices */
        if (ecIndices != NULL) {
            pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(ecIndices);
            extract.filter(*planeCloud);
        }

        viewer.showCloud(planeCloud);


    }

}


void CalibUIApp::extractElements(const Cloud::ConstPtr& planeCloud) {

    /* segment the cloud into euclidiean clusters */
    std::vector<pcl::PointIndices> clusters_indices;

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree 
        (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud (planeCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance (distanceTollerance);
    ec.setMinClusterSize(minElementClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(planeCloud);
    ec.extract(clusters_indices);

    /* for each cluster, check if it is an element */
    for (auto cluster_is : clusters_indices) {

        // this is stupid, euclidean clusters won't give shared pointers,
        // but ExtractIndices needs one.
        boost::shared_ptr<pcl::PointIndices> ciptr(new pcl::PointIndices(cluster_is));
     
        /* extract the cloud */
        Cloud::Ptr elementCloud(new Cloud); 
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud(planeCloud);
        extract.setIndices(ciptr);
        extract.filter(*elementCloud);       

        /* compute the color descriptor for the element */
        ColorDescriptor cdesc;
        cdesc.compute(elementCloud);

        /* use a default element and computed descritpor from the marker */
        if (learnDescriptor) {

            float hueDistance = ColorDescriptor::hueDistance(
                defaultDescriptor.getPrimaryHue(), cdesc.getPrimaryHue());  

            if (hueDistance < maxHueDistance) {
                Element::Ptr element = defaultElementType->createDefaultElement();
                if (element != NULL) {
                    element->defineFromCloud(elementCloud);
                    elementStorage->addElement(element);
                    printAddElement(element);
                }
            }

        } else {  /* find a matching ElementType from the config */

            for (ElementType* etype : elementStorage->getElementTypes()) {

                /* match by hue only */
                float hueDistance = ColorDescriptor::hueDistance(
                    etype->getPrimaryHue(), cdesc.getPrimaryHue());  

                /* create the element */
                if (hueDistance < maxHueDistance) {
                    Element::Ptr element = etype->createDefaultElement();
                    if (element != NULL) {
                        element->defineFromCloud(elementCloud);
                        elementStorage->addElement(element);
                        printAddElement(element);
                    }
                } 

                //TODO check if the normals of the calib plane and the element
                // align to some extent.
            }

        }

    }
}


void CalibUIApp::printAddElement(const Element::Ptr& element) {
    std::cout << "Adding a Element:" << std::endl;
    std::cout << "    Type: " << element->elementTypeName << std::endl;
    std::cout << "    Hull Size: " << element->hullCloud->size() << std::endl;
}


void CalibUIApp::viewerKeyboardCallback(
        const pcl::visualization::KeyboardEvent& ev, void *)
{
    if (ev.getKeySym() == "c") {
        doCaptureMarker = true;
    } else if (ev.getKeySym() == "Escape") {
        doQuit = true;
    }
}   


