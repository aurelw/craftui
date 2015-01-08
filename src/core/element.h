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


#ifndef __ELEMENT_H__
#define __ELEMENT_H__

#include <iostream>
#include <string>
#include <memory>

#include <pcl/common/common_headers.h>
#include <opencv2/core/core.hpp>


class Element {

    public:

        typedef typename std::shared_ptr<Element> Ptr;

        virtual void loadFromFileStorage(const cv::FileNode&);
        virtual void saveToFileStorage(cv::FileStorage&);

        /* naming */
        std::string elementTypeName = "element";
        std::string id = "unnamed";

        /* orientation */

    protected:

};


#endif

