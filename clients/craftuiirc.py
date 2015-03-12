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


import sys
import ssl
import time

import irc.client

from threading import Thread, Lock


class CraftUIIRC:

    def __init__(self, server, port, channel):
        self._thread = None
        self._lock = Lock()

        self.server = server
        self.port = port
        self.channel = channel

        self.isJoined = False

        self._connect()

        self.postLines = []
        self.messageTimestamps = {}



    def start(self):
        if self._thread:
            return
        self._stopThread = False
        self._thread = Thread(target=self.runThread)
        self._thread.start()


    def stop(self):
        thread = self._thread
        if thread:
            self._stopThread = True
            thread.join()
            self._thread = None
            self._stopThread = False


    def postLine(self, line):
        with self._lock:
            self.postLines.append(line)


    def postLinesRateLimited(self, key, minDuration, lines):
        currentTime = time.time()
        if (self.messageTimestamps.has_key(key)):
            lastTime = self.messageTimestamps[key]
            # last post not enough time ago?
            if (lastTime + minDuration > currentTime):
                return
        
        self.messageTimestamps[key] = currentTime
        with self._lock:
            self.postLines.extend(lines)



    def _connect(self):
        c = None
        ssl_factory = irc.connection.Factory(wrapper=ssl.wrap_socket)
        self.reactor = irc.client.Reactor()

        try:
            c = self.reactor.server().connect(
                self.server,
                self.port,
                "CraftUI",
                connect_factory=ssl_factory,
            )
        except irc.reactor.ServerConnectionError:
            print(sys.exc_info()[1])
            return

        c.add_global_handler("welcome", self.on_connect)
        c.add_global_handler("join", self.on_join)
        c.add_global_handler("disconnect", self.on_disconnect)

        self.connection = c


    def runThread(self):
        while not self._stopThread:
            self.reactor.process_once()

            while not self.connection.is_connected() and not self._stopThread:
                self._connect()

            self.handle_messages()
            time.sleep(1)


    def on_connect(self, connection, event):
        if irc.client.is_channel(self.channel):
            connection.join(self.channel)
            return
        self.main_loop(connection)

    def on_join(self, connection, event):
        self.isJoined = True

    def on_disconnect(self, connection, event):
        self.isJoined = False
        print "on disconnect"
        while not self.connection.is_connected():
            print "   try disconnected..."
            self._connect()
            time.sleep(1)
        

    def handle_messages(self):

        if not self.connection.is_connected() or not self.isJoined:
            return

        with self._lock:
            while self.postLines != []:
                self.connection.privmsg(self.channel, self.postLines.pop(0))


def main():

    client = CraftUIIRC("irc.servus.at", 6667, "#test")
    client.start()
    while True:
        time.sleep(5)
        client.postLine("ping")

if __name__ == '__main__':
    main()

