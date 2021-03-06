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


#include "craftuiapp.h"

#include <pcl/filters/plane_clipper3D.h>

#include "craftui_mathutils.h"
#include "craftui_cloudutils.h"
#include "dynamicthreshcalib.h"
#include "eventgenerator.h"


void CraftUIApp::run() {

    EventGenerator eventGen;
    Cloud::Ptr capturedCloud;

    /* calibrate the dynamic noise threshhold */
    DynamicThreshCalib threshCalib(elementStorage->getElements());
    for (int i=0; i<numThreshSamples; i++) {
        updateCloud();
        threshCalib.calibWithCloud(cloud);
        if (withViewer) {
            viewer->showCloud(cloud);
            if (viewer->wasStopped()) return;
        }
    }
    
    /* main loop */
    while (!doQuit) {
        updateCloud(); 

        auto elements = elementStorage->getElements();

        /* do all collisions */
        for (auto element : elements) {
            element->resetCollision();
            element->collideCloud(cloud);
        }

        /* process events */
        for (auto element : elements) {
            element->accept(eventGen);
        }

        if (withViewer) {
            viewer->showCloud(cloud);
            if (viewer->wasStopped()) return;
        }
    }

}


void CraftUIApp::viewerKeyboardCallback(
        const pcl::visualization::KeyboardEvent& ev, void *)
{
    if (ev.getKeySym() == "Escape") {
        doQuit = true;
    }
}   


void CraftUIApp::showHullClouds() {
    if (!withViewer) return;

    int idx = 0;
    for (auto element : elementStorage->getElements()) {
        idx++;
        colorCloud(255, 0, 0, element->hullCloud);
        viewer->showCloud(element->hullCloud, std::to_string(idx++));
    }
}

