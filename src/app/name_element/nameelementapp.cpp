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


#include "nameelementapp.h"

#include <pcl/filters/plane_clipper3D.h>

#include "craftui_mathutils.h"
#include "dynamicthreshcalib.h"
#include "eventgenerator.h"


void NameElementApp::run() {

    Cloud::Ptr capturedCloud;

    /* calibrate the dynamic noise threshold */
    std::cout << "Calibrating dynamic threshold.... ";
    std::cout.flush();
    DynamicThreshCalib threshCalib(elementStorage->getElements());
    for (int i=0; i<numThreshSamples; i++) {
        updateCloud();
        threshCalib.calibWithCloud(cloud);
        if (withViewer) {
            viewer->showCloud(cloud);
            if (viewer->wasStopped()) return;
        }
    }
    std::cout << "done." << std::endl;
    

    /* main loop */
    while (!doQuit) {
        updateCloud(); 

        auto elements = elementStorage->getElements();

        /* do all collisions */
        for (auto element : elements) {
            element->resetCollision();
            element->collideCloud(cloud);
            if (element->isTriggered()) {
                queryElementName(element);
            }
        }

        if (withViewer) {
            viewer->showCloud(cloud);
            if (viewer->wasStopped()) return;
        }
    }

}



void NameElementApp::viewerKeyboardCallback(
        const pcl::visualization::KeyboardEvent& ev, void *)
{
    if (ev.getKeySym() == "Escape") {
        doQuit = true;
    }
}   


void NameElementApp::queryElementName(const Element::Ptr& element) {
    std::string newId = "";
    std::cout << "== Element Selected ==" << std::endl;
    std::cout << "Type: " << element->elementTypeName << std::endl;
    std::cout << "Current ID: " << element->id << std::endl;; 
    std::cout << "Enter new ID:" << std::endl;
    std::cout << "#> ";
    std::getline(std::cin, newId);

    if (newId == "") {
        std::cout << "Keeping old ID: " << element->id << std::endl;
    } else {
        element->id = newId;
        std::cout << "OK." << std::endl;
    }
}

