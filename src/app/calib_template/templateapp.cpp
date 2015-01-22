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
        // only compute from points with a miminum saturation
        cDesc.setMinimumSat(0.2);
        cDesc.compute(planeMarker.markerCloud);

        /* output to console */
        std::cout << "///////// Found Marker /////////" << std::endl;
        std::cout << "Primary Hue: " <<  cDesc.getPrimaryHue() << std::endl;
        std::cout << "Primary Sat: " <<  cDesc.getPrimarySat() << std::endl;
        std::cout << "////////////////////////////////" << std::endl;

        /* get the ElementType to save the descriptor to*/
        while (!elementMenu(cDesc));
        
    }

    while (!doQuit && !viewer.wasStopped()) {
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


bool TemplateApp::elementMenu(const ColorDescriptor& cd) {

    std::vector<ElementType*> etypes = elementStorage->getElementTypes();

    /* print options */
    std::cout << "Assign the color calibration to an ElementType:" << std::endl;
    for (int i=0; i<etypes.size(); i++) {
        std::cout << "[" << i << "] " << etypes[i]->elementname << std::endl;
    }
    std::cout << "[-1] " << "No Type" << std::endl;

    /* read input */
    int index = -2;
    std::string input;
    std::cout << "Select Element: ";
    std::cin >> input;
    try {
        index = std::stoi(input);
    } catch (...) {
        return false;
    }

    if (index == -1) {
        std::cout << "ColorDescriptor is NOT assigned to any ElementType." 
                  << std::endl;
        return true;
    } else if (index >= 0 && index < etypes.size()) {
        etypes[index]->setPrimaryHue( cd.getPrimaryHue() );
        std::cout << "Assigning ColorDescritpor to ElementType " 
                  << etypes[index]->elementname
                  << "." << std::endl;
        return true;
    } else {
        return false;
    }

}


