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

#include "colordescriptor.h"


int main(int argc, char **argv) {

#ifdef linux
    std::cout << "'linux' is defined" << std::endl;
#endif

#if linux
    std::cout << "linux == true" << std::endl;
#endif

    pcl::PointXYZRGBA p;
    p.r = 155;
    p.g = 15;
    p.b = 12;
    
    std::cout << "RGB: " << p.r << ", " << p.g << ", " << p.b << std::endl ;
    std::cout << "Point: " << p << std::endl ;

    float h, s, v;
    ColorDescriptor::pointToHSV(p, h, s, v);

    std::cout << "HSV: " << h << ", " << s << ", " << v << std::endl;

}

