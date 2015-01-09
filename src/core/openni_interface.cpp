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

#include "openni_interface.h"


OpenNiInterface::~OpenNiInterface() {
    grabber->stop();
    //FIXME wait for grabber to stop
    sleep(1);
}


void OpenNiInterface::cloud_callback(const Cloud::ConstPtr &cld) {
    //FIXME possibly good to use thread safety check if updateSignal()
    // runs in a different thread.
    cloud = cld;
    if (cloud != NULL) {
        isStreaming = true;
        updateSignal();
    }
}


void OpenNiInterface::setupGrabber() {

    // modes can be specified
    pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
    pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

    try {
        //FIXME deviceId
        grabber = new pcl::OpenNIGrabber();
    } catch (pcl::IOException &exc) {
        return;
    }
    
    /* callbacks XYZRGBA */
    boost::function <void (const Cloud::ConstPtr&)> cloud_cb = 
        boost::bind (&OpenNiInterface::cloud_callback, this, _1);
    grabber->registerCallback(cloud_cb);

    // start 
    grabber->start();
    isConnected = true;
}


bool OpenNiInterface::init() {
    setupGrabber();
    return isConnected;
}


void OpenNiInterface::waitForFirstFrame() {
    while (!isStreaming) {
        usleep(10);
    }
}


OpenNiInterface::Cloud::ConstPtr OpenNiInterface::getLastCloud() {
    return cloud;
}


OpenNiInterface::Cloud::Ptr OpenNiInterface::getCloudCopy() {
    Cloud::Ptr ncloud(new Cloud (*getLastCloud()));
    return ncloud;
}


OpenNiInterface::connection_t OpenNiInterface::connect
    (signal_t::slot_function_type subscriber) 
{
    return updateSignal.connect(subscriber);
}

