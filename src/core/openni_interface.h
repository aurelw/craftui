/*
   * (C) 2015, Aurel Wildfellner
   *
   * This file is part of Beholder.
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
   * along with Beholder. If not, see <http://www.gnu.org/licenses/>. */

#ifndef __OPENNI_INTERFACE_H__
#define __OPENNI_INTERFACE_H__

#include <stdio.h>
#include <iostream>
#include <vector>

#include <boost/signals2.hpp>
#include <boost/bind.hpp>
#include <boost/signals2/connection.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// it seems that the platform is undefined
#define linux 1
#define __x86_64__ 1
#include <pcl/io/openni_grabber.h>


class OpenNiInterface {

    public:

        typedef typename pcl::PointCloud<pcl::PointXYZRGBA> Cloud;

        typedef typename  std::shared_ptr<OpenNiInterface> Ptr;


        OpenNiInterface(std::string device) :
            isStreaming(false),
            isConnected(false),
            deviceId(device)
        {
        }

        ~OpenNiInterface();

        bool init();
        void waitForFirstFrame();

        Cloud::ConstPtr getLastCloud();
        Cloud::Ptr getCloudCopy();

        /* update signal */
        typedef boost::signals2::signal<void ()>  signal_t;
        typedef boost::signals2::connection  connection_t;
        connection_t connect(signal_t::slot_function_type subscriber);


    protected:

        void setupGrabber();
        virtual void cloud_callback(const Cloud::ConstPtr& cld);
        pcl::OpenNIGrabber *grabber;
        Cloud::ConstPtr cloud;
        bool isStreaming;
        bool isConnected;
        std::string deviceId;

        signal_t updateSignal;
};


#endif
