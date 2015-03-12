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

import unicodedata

import zmq

import uievents_pb2



class EventSubscriber:

    def __init__(self, address):
        self.addr = address
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.connected = False


    def connect(self):
        self.socket.connect(self.addr)
        # IMPORTANT subscribe to all messages
        self.socket.setsockopt_string(zmq.SUBSCRIBE, u"")
        self.connected = True


    def receiveEvent(self):
        if not self.connected:
            return None

        msg = self.socket.recv_string()
        msgstr = unicodedata.normalize('NFKD', msg).encode('ascii','ignore')
        event = uievents_pb2.Event()
        event.ParseFromString(msgstr)
        return event


