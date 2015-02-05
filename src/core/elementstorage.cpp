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

#include "time.h"

#include "elementstorage.h"


ElementStorage::ElementStorage() {
    elementTypes.push_back(&buttonType);
    elementTypes.push_back(&sliderType);
    elementTypes.push_back(&calibSquareType);
}


bool ElementStorage::loadFromFile(const std::string& path) {

    /* clear old storage */
    elements.clear();

    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::READ);
    loadMeta(fs);   
    loadElementTypes(fs);
    loadElements(fs);
    fs.release();

    return true;
}


void ElementStorage::loadMeta(const cv::FileStorage& fs) {
    cv::FileNode metanode = fs["MetaData"];
}


void ElementStorage::loadElementTypes(const cv::FileStorage& fs) {
    cv::FileNode tnode = fs["ElementTypes"];
    buttonType.loadFromFileStorage(tnode[buttonType.elementname]);
    sliderType.loadFromFileStorage(tnode[sliderType.elementname]);
    calibSquareType.loadFromFileStorage(tnode[calibSquareType.elementname]);
}


void ElementStorage::loadElements(const cv::FileStorage& fs) {
    cv::FileNode elements_node = fs["Elements"];
    cv::FileNodeIterator it = elements_node.begin(), it_end = elements_node.end();

    for (; it != it_end; it++) {

        /* create the concrete element based on the typename */
        std::string tname = (*it)["typename"];
        Element::Ptr element;
        for (auto etype : elementTypes) {
            if (etype->elementname == tname) {
                element = etype->createDefaultElement();
                break;
            }
        }

        element->loadFromFileStorage(*it);
        elements.push_back(element);
    }

}


bool ElementStorage::saveToFile(const std::string& path) {
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::WRITE);
    saveMeta(fs);
    saveElementTypes(fs);
    saveElements(fs);
    fs.release();

    return true;
}


void ElementStorage::saveMeta(cv::FileStorage& fs) {
    fs << "MetaData" << "{";

    /* a timestamp */
    time_t timeObj;
    time(&timeObj);
    tm *pTime = gmtime(&timeObj);
    char buffer[100];
    sprintf(buffer, "%.2d-%.2d-%d %.2d:%.2d:%.2d",
            pTime->tm_mday, pTime->tm_mon+1, pTime->tm_year+1900,
            pTime->tm_hour, pTime->tm_min, pTime->tm_sec);
    fs << "timestamp" << buffer; 

    fs << "}";
}


void ElementStorage::saveElementTypes(cv::FileStorage& fs) {
    fs << "ElementTypes" << "{";
    buttonType.saveToFileStorage(fs);
    sliderType.saveToFileStorage(fs);
    calibSquareType.saveToFileStorage(fs);
    fs << "}";
}


void ElementStorage::saveElements(cv::FileStorage& fs) {

    fs << "Elements" << "{";

    /* create a node for each element and store the element within */
    for (int i = 0; i<elements.size(); i++) {
        fs << "elemnt" + std::to_string(i) << "{";
        elements[i]->saveToFileStorage(fs);
        fs << "}";
    }

    fs << "}";
}


void ElementStorage::addElement(const Element::Ptr& element) {
    elements.push_back(element);
}


std::vector<Element::Ptr> ElementStorage::getElements() const {
    return elements;
}


std::vector<ElementType*> ElementStorage::getElementTypes() const {
    return elementTypes;
}

