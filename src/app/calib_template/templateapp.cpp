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


#include "templateapp.h"

#include "colordescriptor.h"


void TemplateApp::run() {

    bool foundMarker = false;

    while (!foundMarker) {
        updateCloud(); 
        viewer.showCloud(cloud);

        if (doCaptureMarker) {
            foundMarker = planeMarker.computeMarkerCenter(cloud, markerPoint);
            break;
        }

        if (doQuit) {
            return;
        }
    }

    if (foundMarker) {
        viewer.showCloud(planeMarker.markerCloud);
        ColorDescriptor cDesc;
        cDesc.compute(planeMarker.markerCloud);
        std::cout << "Primary Hue: " <<  cDesc.getPrimaryHue() << std::endl;
        std::cout << "Primary Sat: " <<  cDesc.getPrimarySat() << std::endl;
    }

    while (!doQuit) {
        sleep(0.2);
    }
}


void TemplateApp::viewerKeyboardCallback(
        const pcl::visualization::KeyboardEvent& ev, void *)
{
    if (ev.getKeySym() == "c") {
        doCaptureMarker = true;
    } else if (ev.getKeySym() == "Escape") {
        doQuit = true;
    }
}   

