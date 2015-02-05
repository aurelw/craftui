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


#ifndef __CRAFTUI_APP_H__
#define __CRAFTUI_APP_H__

#include <boost/bind.hpp>

#include <pcl/visualization/cloud_viewer.h>

#include "openni_interface_connection.h"
#include "elementstorage.h"
#include "plane_marker.h"
#include "colordescriptor.h"
#include "elementtype.h"


class CraftUIApp : public OpenNiInterfaceConnection {

    public:

        typedef typename OpenNiInterface::Cloud Cloud;

        CraftUIApp(const OpenNiInterface::Ptr& oniif,
                    const ElementStorage::Ptr& estore,
                    bool with3DUI=true) :
            OpenNiInterfaceConnection(oniif),
            elementStorage(estore),
            withViewer(with3DUI)
        {

            if (withViewer) {
                viewer.reset(new pcl::visualization::CloudViewer("CraftUI"));
                viewer->registerKeyboardCallback(
                        &CraftUIApp::viewerKeyboardCallback, *this);
            }

        }

        void run();

        void viewerKeyboardCallback(
                const pcl::visualization::KeyboardEvent &, void *);

    private:

        bool withViewer;
        std::shared_ptr<pcl::visualization::CloudViewer> viewer;
        bool doQuit = false;

        int numThreshSamples = 100;

        ElementStorage::Ptr elementStorage;
};

#endif

