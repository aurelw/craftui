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


#ifndef __CALIBSQUARE_TYPE_H__
#define __CALIBSQUARE_TYPE_H__

#include "elementtype.h"


class CalibSquareType : public ElementType {

    public:

        virtual void loadFromFileStorage(const cv::FileNode& node) override;
        virtual void saveToFileStorage(cv::FileStorage&) override;
        
        const std::string elementname = "calibsquare";

    protected:

        // A sheet of A4 paper
        float width = 0.297;
        float height = 0.210;
};


#endif

