#!/usr/bin/python

###
# Copyright 2015, Aurel Wildfellner.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#

import re
import os
import sys

from craftui_eventsubscriber import EventSubscriber


class CraftUIMQTTPub:

    def __init__(self, broker_host="localhost", craftui_url="tcp://127.0.0.1:9001"):
        self.baseTopic = "craftui/"
        self.brokerHost = broker_host

        self.eventSubscriber = EventSubscriber(craftui_url)
        self.eventSubscriber.connect()

    def run(self):

        while True:
            event = self.eventSubscriber.receiveEvent()

            # button events
            if event.elementtype == event.BUTTON:
                topic = "button/" + event.id
                if event.trigger == event.TRIGGERED:
                    msg = "DOWN"
                elif event.trigger == event.UNTRIGGERED:
                    msg = "UP"
                self.pub(topic, msg)

    def pub(self, topic, msg):
        fulltopic = self.baseTopic + topic
        #FIXME use python-mosquitto
        os.system("mosquitto_pub -h " + self.brokerHost + 
                " -t " + fulltopic + " -m " + msg )
        print(fulltopic + " " + msg)



def main():
    
    brokerhost = "localhost"
    if (len(sys.argv) > 1):
        brokerhost = sys.argv[1]

    craftuiurl = "tcp://127.0.0.1:9001"
    if (len(sys.argv) > 2):
        craftuiurl = sys.argv[2]

    sys.stdout.write("Connecting client to CraftUI.... ")
    client = CraftUIMQTTPub(brokerhost, craftuiurl)
    print("done.")
    client.run()


if __name__ == "__main__":
    main()

