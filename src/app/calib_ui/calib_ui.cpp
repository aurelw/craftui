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

#include <pcl/console/parse.h>

#include "elementstorage.h"
#include "plane_marker.h"
#include "calibuiapp.h"
#include "openni_interface.h"


void print_usage() {
    std::cout << "craftui_calib_ui -c <config> [-t <ElementType>] [--max-hue-distance=<hue>]" << std::endl;
    std::cout << std::endl;
    std::cout << "   -t <ElementType>    Learn the color descriptor from the calibration" << std::endl;
    std::cout << "                       pattern and assume all found markers to be of" << std::endl;
    std::cout << "                       the given type." << std::endl;
    std::cout << "   --max-hue-distance=<hue>" << std::endl;
    std::cout << "                       The maximum hue distance between two matching" << std::endl;
    std::cout << "                       color descriptors." << std::endl;
}


int main(int argc, char **argv) {

    if ( pcl::console::find_switch(argc, argv, "-h") ||
         pcl::console::find_switch(argc, argv, "--help"))
    {
        print_usage();
        exit(0);
    }
    
    /* setup element storage */
    std::string filepath;
    if (pcl::console::parse_argument(argc, argv, "-c", filepath) == -1) {
        std::cerr << "[ERROR] No config file provided!" << std::endl;
        print_usage();
        exit(1); 
    }

    /* load config / element storage */
    ElementStorage::Ptr eStore(new ElementStorage());
    bool success = eStore->loadFromFile(filepath);
    if (!success) {
        std::cerr << "[ERROR] Couldn't write to file!" << std::endl;
        exit(1);
    }

    /* learn descriptor for element with <typeName> from calib patter */
    bool learnDescriptor = false;
    std::string elementTypeName = "";
    ElementType* elementType = NULL;
    if (pcl::console::parse_argument(argc, argv, "-t", elementTypeName) != -1) {
        /* Get an ElementType with such a name */
        std::vector<ElementType*> eTypes = eStore->getElementTypes();
        for (auto eType : eTypes) {
            if (eType->elementname == elementTypeName) {
                elementType = eType;
                learnDescriptor = true;
            }
        } 
        if (elementType == NULL) {
            std::cerr << "[ERROR] No ElementType '" << elementTypeName << "'" << std::endl;
            print_usage();
            exit(1);
        }
    }


    /* specify the maximum hue distance between matching color descriptors */
    float maxHueDistance = 10.0f;
    /* if the color descriptor is learned from the same frame, 
     * a lower matching distance is assumed. */
    if (learnDescriptor) {
        maxHueDistance = 5.0f;
    }
    pcl::console::parse_argument(argc, argv, "--max-hue-distance", maxHueDistance);

    /* setup the kinect interface */
    OpenNiInterface::Ptr openNiIf(new OpenNiInterface("0"));
    openNiIf->init();
    openNiIf->waitForFirstFrame();

    /* create the app and run it */
    CalibUIApp app(openNiIf, eStore, learnDescriptor, elementType, maxHueDistance);
    app.run();

    /* save calibration results to element storage */
    eStore->saveToFile(filepath);
    std::cout << "Saved to file." << std::endl;

    exit(0);
}

