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
import time
import datetime
import argparse

from craftui_eventsubscriber import EventSubscriber


def fullEventString(event):
        ### Print Event Info ###
        s = "########################\n"
        s += event.id + " - " + str(datetime.datetime.now()) + "\n"
        if event.elementtype == event.BUTTON:
            s += "  Type: BUTTON\n" 
        elif event.elementtype == event.SLIDER:
            print "  Type: SLIDER\n" 

        if event.trigger == event.TRIGGERED:
            print "  Triggered!\n"
        elif event.trigger == event.UNTRIGGERED:
            print "  Untriggered!\n"
        elif event.trigger == event.INTRIGGER:
            print "  Intrigger.\n"

        if s[-1:] == '\n':
            return s[:-1]
        else:
            return s

def shortEventString(event):
    s = "[" + str(datetime.datetime.now()) + "] "
    s += event.id + ": "
    if event.trigger == event.TRIGGERED:
        s += "triggered."
    elif event.trigger == event.UNTRIGGERED:
        s += "untrigdered."
    elif event.trigger == event.INTRIGGER:
        s += "intrigger."
    return s


def printHelp():
    print "craftui_print_events [--host <host>] [-p <port>] [--short] [--logfile <filename>]"



def main():

    ## Parse Command Line Arguments
    if "-h" in sys.argv or "--help" in sys.argv:
        printHelp()

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("-p", type=int, default=9001)
    parser.add_argument("--short", default=False, action="store_true")
    parser.add_argument("--logfile")
    args = parser.parse_args() 

    printShort = args.short

    doLogfile = (args.logfile != None)
    fname = args.logfile

    ## Connect to CraftUI
    hostUrl = "tcp://" + args.host + ":" + str(args.p)
    sys.stdout.write("Connecting.... ") 
    evs = EventSubscriber("tcp://127.0.0.1:9001")
    evs.connect()
    if evs.connected:
        print "done."
    else:
        print "ERROR."
        sys.exit(1)

    ## Open the logfile in append mode.
    logfile = None
    if doLogfile:
        try:
            logfile = open(fname, "a")
            print "Appending to logfile '" + fname + "'"
        except:
            print "Error accesing logfile."
            exit(1)

    ## Process events
    while True:
        event = evs.receiveEvent()
        if printShort:
            s = shortEventString(event)
        else:
            s = fullEventString(event)

        print(s)
        if logfile:
            logfile.write(s)
            logfile.write("\n")


    # Close the file
    if logfile:
        logfile.close()


if __name__ == "__main__":
    main()

