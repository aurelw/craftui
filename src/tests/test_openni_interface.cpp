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
#include <pcl/io/pcd_io.h>


#include "openni_interface.h"
#include "openni_interface_connection.h"


class TestCloudConsumer : public OpenNiInterfaceConnection {

    public:

        TestCloudConsumer(const OpenNiInterface::Ptr& oniif) :
            OpenNiInterfaceConnection(oniif)
        {
        }

        OpenNiInterface::Cloud::Ptr getNextCloud() {
            updateCloud();
            return cloud;
        }


};


int main(int argc, char **argv) {

    OpenNiInterface::Ptr openniif(new OpenNiInterface("0"));
    openniif->init();
    std::cout << "Init done." << std::endl;
    openniif->waitForFirstFrame();
    std::cout << "Streaming Good." << std::endl;

    TestCloudConsumer consumer(openniif);
    pcl::io::savePCDFileASCII ("test_captured.pcd", *(consumer.getNextCloud()));

    std::cout << "Capture Good." << std::endl;
    sleep(3);

}

