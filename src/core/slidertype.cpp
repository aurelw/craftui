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

#include "slidertype.h"


void SliderType::loadFromFileStorage(const cv::FileNode& node) {
    ElementType::loadFromFileStorage(node);
    node["numPointsThresh"] >> numPointsThresh;
    node["maxDistance"] >> maxDistance;
}


void SliderType::saveToFileStorage(cv::FileStorage& fs) {
    fs << elementname << "{";
    ElementType::saveToFileStorage(fs);
    fs << "numPointsThresh" << numPointsThresh;
    fs << "maxDistance" << maxDistance;
    fs << "}";
}


Element::Ptr SliderType::createDefaultElement() {
    Slider::Ptr slider = createDefaultSlider();
    Element::Ptr element(slider);
    return element;
}

Slider::Ptr SliderType::createDefaultSlider() {
    Slider::Ptr slider(new Slider());
    slider->elementTypeName = elementname;
    slider->numPointsThresh = numPointsThresh;
    slider->maxDistance = maxDistance;
    return slider;
}

