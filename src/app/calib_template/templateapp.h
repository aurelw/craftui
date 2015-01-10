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


#ifndef __TEMPLATE_APP_H__
#define __TEMPLATE_APP_H__

#include <boost/bind.hpp>

#include <pcl/visualization/cloud_viewer.h>

#include "openni_interface_connection.h"
#include "elementstorage.h"
#include "plane_marker.h"


class TemplateApp : public OpenNiInterfaceConnection {

    public:

        TemplateApp(const OpenNiInterface::Ptr& oniif,
                    const ElementStorage::Ptr& estore) :
            OpenNiInterfaceConnection(oniif),
            elementStorage(estore),
            viewer("Cloud Viewer")
        {

            /* setup plane marker */
            float length = elementStorage->calibSquareType.length;
            float width = elementStorage->calibSquareType.width;
            planeMarker.setMarkerDimensions(length, width, 0.05);

            /* setup viewer */
            viewer.registerKeyboardCallback(&TemplateApp::viewerKeyboardCallback, *this);

        }

        void run();

        void viewerKeyboardCallback(
                const pcl::visualization::KeyboardEvent &, void *);

    private:

        ElementStorage::Ptr elementStorage;
        PlaneMarker<pcl::PointXYZRGBA> planeMarker;

        pcl::visualization::CloudViewer viewer;
        pcl::PointXYZRGBA markerPoint;
        /* flags for keyboard commands */
        bool doCaptureMarker = false;
        bool doQuit = false;

};


#endif

