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


#ifndef __BUTTON_TYPE_H__
#define __BUTTON_TYPE_H__

#include "elementtype.h"
#include "button.h"


class ButtonType : public ElementType {

    public:

        ButtonType() {
            ElementType::elementname = "button";
        };

        virtual void loadFromFileStorage(const cv::FileNode& node) override;
        virtual void saveToFileStorage(cv::FileStorage&) override;

        virtual Element::Ptr createDefaultElement() override;
        virtual Button::Ptr createDefaultButton();
        

    protected:

        // minimum number of inside the volume to register a press
        int numPointsThresh = 60;
        // maximum distance in meters from the element to press the button
        float maxDistance = 0.03;

};


#endif

