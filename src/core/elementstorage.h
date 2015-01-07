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


#ifndef __ELEMENT_STORAGE_H__
#define __ELEMENT_STORAGE_H__

#include "buttontype.h"
#include "calibsquaretype.h"
#include "slidertype.h"


class ElementStorage {

    public:

        bool loadFromFile(const std::string& path);
        bool saveToFile(const std::string& path);


        /* element types */
        ButtonType buttonType;
        SliderType sliderType;
        CalibSquareType calibSquareType;


    private:

    void loadMeta(const cv::FileStorage&);
    void loadElementTypes(const cv::FileStorage&);
    void saveMeta(cv::FileStorage&);
    void saveElementTypes(cv::FileStorage&);

};


#endif

