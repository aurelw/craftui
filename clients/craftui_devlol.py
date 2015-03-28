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

import craftuiirc
from craftui_eventsubscriber import EventSubscriber



def setProjMappingColor(color, host):
    os.system("mosquitto_pub -h " + host + " -t beamer/projmap/color -m " + color)

def setLolStripe(color, host):
    os.system("mosquitto_pub -h " + host + " -t led -m x" + color*60 )

def displayHi5(host):
    os.system("mosquitto_pub -h " + host + " -t lolshield/oneshot -m \"  Hi5!!   Hi5!!   Hi5!! \"")

def displayHelloOnIRC(host):
    os.system("mosquitto_pub -h " + host + " -t lolshield/oneshot -m \"CRAFTUI:  You just posted Hello to our IRC channel!\"")


def toggleChico(port):
    ports = str(port) 
    ports = re.escape(ports)
    os.system('sispmctl -t ' + ports + " > /dev/null")


def main():

    brokerhost = "localhost"
    if (len(sys.argv) > 1):
        brokerhost = sys.argv[1]

    evs = EventSubscriber("tcp://127.0.0.1:9001")
    evs.connect()
    print("Connected: ", evs.connected)

    ircclient = craftuiirc.CraftUIIRC("irc.servus.at", 6667, "#devlol")
    ircclient.start()
    

    while True:
        event = evs.receiveEvent()

        ### Print Event Info ###
        print "########################"
        print event.id + ":"
        if event.elementtype == event.BUTTON:
            print "  Type: BUTTON" 
        elif event.elementtype == event.SLIDER:
            print "  Type: SLIDER" 
        if event.trigger == event.TRIGGERED:
            print "  Triggered!"
        elif event.trigger == event.UNTRIGGERED:
            print "  Untriggered!"
        elif event.trigger == event.INTRIGGER:
            print "  Intrigger."

        ### The LoL strip ###
        if event.id == "button_red" and event.trigger == event.TRIGGERED:
            setLolStripe("R", brokerhost)
            setProjMappingColor("R", brokerhost)
        if event.id == "button_green" and event.trigger == event.TRIGGERED:
            setLolStripe("G", brokerhost)
            setProjMappingColor("G", brokerhost)
        if event.id == "button_blue" and event.trigger == event.TRIGGERED:
            setLolStripe("B", brokerhost)
            setProjMappingColor("B", brokerhost)

        ### Print to IRC ###
        if event.id == "button_black" and event.trigger == event.TRIGGERED:
            ircclient.postLinesRateLimited("hi", 10, ["Someone says Hi at the window!"])
            displayHelloOnIRC(brokerhost)
        if event.id == "buttonHi5" and event.trigger == event.TRIGGERED:
            ircclient.postLinesRateLimited("hi5", 10, ["Hi 5!"])
            displayHi5(brokerhost)
            #toggleChico(2)

    ircclient.stop()





if __name__ == "__main__":
    main()

