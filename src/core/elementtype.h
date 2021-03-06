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


#ifndef __ELEMENT_TYPE_H__
#define __ELEMENT_TYPE_H__


#include <pcl/common/common_headers.h>

#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>

#include "element.h"


class ElementType {

    public:

        virtual void loadFromFileStorage(const cv::FileNode&) = 0;
        virtual void saveToFileStorage(cv::FileStorage&) = 0;

        virtual Element::Ptr createDefaultElement() = 0;

        /* the primary Hue for this element type */
        void setPrimaryHue(const float hue);
        float getPrimaryHue() const;

        std::string elementname = "element";

    protected:

        float primaryHue = 0.0;

};


#endif

