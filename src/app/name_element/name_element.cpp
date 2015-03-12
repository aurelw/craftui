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

#include "nameelementapp.h"
#include "elementstorage.h"
#include "openni_interface.h"


void print_usage() {
    std::cout << "craftui_name_element -c <config> [--noui]" << std::endl;
}


int main(int argc, char **argv) {

    if ( pcl::console::find_switch(argc, argv, "-h") ||
         pcl::console::find_switch(argc, argv, "--help"))
    {
        print_usage();
        exit(0);
    }

    bool with3DUI = ! (pcl::console::find_switch(argc, argv, "--noui"));
    
    /* setup element storage */
    std::string filepath;
    if (pcl::console::parse_argument(argc, argv, "-c", filepath) == -1) {
        std::cerr << "[ERROR] No config file provided!" << std::endl;
        exit(1); 
    }

    ElementStorage::Ptr eStore(new ElementStorage());
    bool success = eStore->loadFromFile(filepath);
    if (!success) {
        std::cerr << "[ERROR] Couldn't read from file!" << std::endl;
        exit(1);
    }

    /* setup the kinect interface */
    OpenNiInterface::Ptr openNiIf(new OpenNiInterface("0"));
    openNiIf->init();
    openNiIf->waitForFirstFrame();

    /* create the app and run it */
    NameElementApp app(openNiIf, eStore, with3DUI);
    app.run();

    /* store the element names */
    eStore->saveToFile(filepath);
    std::cout << "Saved element storagage to file." << std::endl;

    exit(0);
}

