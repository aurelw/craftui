#!/usr/bin/python3

import time
import datetime

import sys
import argparse


def readConfig(filename):
    events = []
    with open(filename) as f:
        for s in f:
            try:
                ts = s.split("]")[0][1:].split(".")[0]
                eventtime = datetime.datetime.strptime(ts, "%Y-%m-%d %H:%M:%S")
                eventtrigger = False if s.find("untrigdered") != -1 else True
                eventbutton = s.split("]")[1].split(":")[0].strip()
                events.append( (eventtime, eventbutton, eventtrigger) )
            except:
                pass

    return events


def cluserEvents(events, minsecdist=1.0):

    clusters = []
    clusters.append( events[0] )

    ctime = events[0][0]
    for event in events[1:]:
        delta = event[0] - ctime
        if delta.total_seconds() > minsecdist:
            clusters.append(event)
        ctime = event[0]

    return clusters


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("-f", type=str, help="Input log file.")
    parser.add_argument("--clusterdist", type=float, default=1.0)
    args = parser.parse_args()

    if not args.f:
        print("[ERROR] No input log file.")
        exit(1)

    filename = args.f
    cluster_distance = args.clusterdist

    events = readConfig(filename)
    print("num events: ", len(events))
    clusters = cluserEvents(events, cluster_distance)
    print("num clusters: ", len(clusters))


main()






