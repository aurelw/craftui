/*
   * (C) 2013, Aurel Wildfellner
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

#ifndef __OPENNI_INTERFACE_CONNECTION_H__
#define __OPENNI_INTERFACE_CONNECTION_H__

#include <boost/thread.hpp>
#include <boost/bind.hpp>


#include "openni_interface.h"

class OpenNiInterfaceConnection {

    public:

        OpenNiInterfaceConnection(const OpenNiInterface::Ptr& oniif) {
            openNiIf = oniif;
            connection = openNiIf->connect(
                boost::bind(&OpenNiInterfaceConnection::updateSlot, this));
        }

        ~OpenNiInterfaceConnection() {
            {
                boost::unique_lock<boost::mutex> lock(back_cloud_mutex);
                connection.disconnect();
            }
            //FIXME this waits for callback in OpenNiInterface to be finished
            sleep(1);
        }


    protected:

        /* call this to update 'cloud' with a new cloud 
         * from the grabber interface */
        void updateCloud();
        OpenNiInterface::Cloud::Ptr cloud;

    private:

        OpenNiInterface::Ptr openNiIf;
        OpenNiInterface::connection_t connection;
        void updateSlot();
        OpenNiInterface::Cloud::Ptr back_cloud;
        boost::mutex back_cloud_mutex;
        bool back_cloud_updated = false;
        boost::condition_variable updatedCondition;


};


#endif
