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


#ifndef __CALIB_UI_APP_H__
#define __CALIB_UI_APP_H__

#include <boost/bind.hpp>

#include <pcl/visualization/cloud_viewer.h>

#include "openni_interface_connection.h"
#include "elementstorage.h"
#include "plane_marker.h"
#include "colordescriptor.h"
#include "elementtype.h"


class CalibUIApp : public OpenNiInterfaceConnection {

    public:

        typedef typename OpenNiInterface::Cloud Cloud;

        CalibUIApp(const OpenNiInterface::Ptr& oniif,
                    const ElementStorage::Ptr& estore,
                    const bool learnDescriptor,
                    ElementType* defaultElementType,
                    float maxHueDistance=10.0f) :
            OpenNiInterfaceConnection(oniif),
            elementStorage(estore),
            viewer("Cloud Viewer"),
            learnDescriptor(learnDescriptor),
            defaultElementType(defaultElementType),
            maxHueDistance(maxHueDistance)
        {

            /* setup plane marker */
            float length = elementStorage->calibSquareType.length;
            float width = elementStorage->calibSquareType.width;
            planeMarker.setMarkerDimensions(length, width, length*0.1);

            /* setup viewer */
            viewer.registerKeyboardCallback(&CalibUIApp::viewerKeyboardCallback, *this);

        }

        void run();

        void viewerKeyboardCallback(
                const pcl::visualization::KeyboardEvent &, void *);

    private:

        pcl::visualization::CloudViewer viewer;
        pcl::PointXYZRGBA markerPoint;
        /* flags for keyboard commands */
        bool doCaptureMarker = false;
        bool doQuit = false;

        ElementStorage::Ptr elementStorage;

        /* learn color descriptor from calib pattern */
        bool learnDescriptor;
        ElementType* defaultElementType;
        ColorDescriptor defaultDescriptor;


        PlaneMarker<pcl::PointXYZRGBA> planeMarker;

        void extractElements(const Cloud::ConstPtr& planeCloud);
        void printAddElement(const Element::Ptr& element);

        /* some calibration parameters */
        // the maximum distance between two elements
        float distanceTollerance = 0.015;
        // the maximum difference in hue to match an ElementType
        float maxHueDistance = 10;
        // the tolerance for the distance of points from the UI plane 
        // defined by the calib pattern
        float uiPlaneTollerance = 0.04;
        // cluster constraints for a UI element marker
        float minElementClusterSize = 100;
};


#endif

