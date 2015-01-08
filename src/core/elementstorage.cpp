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


#include "elementstorage.h"

#include "time.h"

bool ElementStorage::loadFromFile(const std::string& path) {
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::READ);
    loadMeta(fs);   
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


bool ElementStorage::saveToFile(const std::string& path) {
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::WRITE);
    saveMeta(fs);
    saveElementTypes(fs);
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

