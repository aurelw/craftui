#!/usr/bin/python3

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

import time
import os
import argparse

import mosquitto


def setColor(color):
    os.system("killall feh")
    if color == "red":
        os.system("feh -F devlol-r.png &")
    elif color == "green":
        os.system("feh -F devlol-g.png &")
    elif color == "blue":
        os.system("feh -F devlol-b.png &")




class MQTT_TOPICS:
    color = "beamer/projmap/color"


def on_message(client, lolshield, msg):
    payload = msg.payload.decode("utf-8")
    """ Callback for mqtt message."""
    if msg.topic == MQTT_TOPICS.color:
        if "R" in payload:
            setColor("red")
        elif "G" in payload:
            setColor("green")
        elif "B" in payload:
            setColor("blue")


def main():

    ## Command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.7.2")

    args = parser.parse_args() 
    brokerHost = args.host
    
    ## setup MQTT client
    client = mosquitto.Mosquitto()
    client.connect(brokerHost)
    client.on_message = on_message

    ## subscribe to topics
    client.subscribe(MQTT_TOPICS.color)

    #client.user_data_set(lolShield)

    while True:
        
        client.loop()


if __name__ == "__main__":
    main()

