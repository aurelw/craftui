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


int main(int argc, char **argv) {

    std::string filepath_save = "test_element_storage_save.xml";
    std::string filepath_loaded = "test_element_storage_loaded.xml";

    /* create a element storage and fill it with dummy elements */
    ElementStorage eStore;

    Element::Ptr e0(new Element);
    Element::Ptr e1(new Element);
    Element::Ptr e2(new Element);

    eStore.addElement(e0);
    eStore.addElement(e1);
    eStore.addElement(e2);

    /* save to file */
    bool success = eStore.saveToFile(filepath_save);
    if (!success) {
        std::cerr << "[ERROR] Couldn't write to file!" << std::endl;
        exit(1);
    }

    /* reopen and store to a different file */
    eStore.loadFromFile(filepath_save);
    eStore.saveToFile(filepath_loaded);

    exit(0);
}

