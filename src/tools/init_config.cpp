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


void print_usage() {
    std::cout << "craftui_init_config <outputfile>" << std::endl;
}


int main(int argc, char **argv) {

    if ( argc != 2) {
        std::cerr << "[ERROR] Wrong number of arguments!" << std::endl;
        print_usage();
        exit(1);
    }

    if ( pcl::console::find_switch(argc, argv, "-h") ||
         pcl::console::find_switch(argc, argv, "--help"))
    {
        print_usage();
    }
    
    std::string filepath = argv[1];
    ElementStorage eStore;
    bool success = eStore.saveToFile("test.xml");
    if (!success) {
        std::cerr << "[ERROR] Couldn't write to file!" << std::endl;
        exit(1);
    }

    exit(0);
}

