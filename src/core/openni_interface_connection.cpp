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

#include "openni_interface_connection.h"


void OpenNiInterfaceConnection::updateSlot() {
    OpenNiInterface::Cloud::Ptr newcloud = openNiIf->getCloudCopy();
    {
        {
            boost::unique_lock<boost::mutex> lock(back_cloud_mutex);
            back_cloud = newcloud;
            back_cloud_updated = true;
        }
        updatedCondition.notify_all();
    }
}


void OpenNiInterfaceConnection::updateCloud() {
    boost::unique_lock<boost::mutex> lock(back_cloud_mutex);
    while (!back_cloud_updated) {
        updatedCondition.wait(lock);
    }
    cloud = back_cloud;   
    back_cloud_updated = false;
}

