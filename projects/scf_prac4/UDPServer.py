# -*- coding: utf-8 -*-

import socket
import sys
import threading
import pickle
import struct
import logging
import time

class UDPServer(threading.Thread):
    def __init__(self, event = None, host = "0.0.0.0", udp_port = 5002):
        threading.Thread.__init__(self)

        self.log = logging.getLogger("UDPServer")

        self.event = event
        
        self.host = host       
        self.udp_port = udp_port
        self.udp_socket = None
        self.last_packet = None

        self.is_finished = False
 
    def exit(self):
        self.log.debug("Exitting")
        self.is_finished = True
   
    def get_last_packet(self):
        return self.last_packet

    def run(self):
        self.log.debug("Starting")

        # Create UDP socket
        self.udp_socket = socket.socket(socket.AF_INET,  socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.setblocking(0)

        # Bind UDP socket
        try:
            self.udp_socket.bind((self.host, self.udp_port))
        except socket.error:
            self.log.debug("Bind failed %s" % (socket.error))

        self.log.info("Waiting for data on UDP port %s" % (self.udp_port))
        
        while not self.is_finished:
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                if (len(data) > 0):
                    self.log.debug("Received packet with length %s", len(data))
                    self.last_packet = struct.unpack('<' + 'h' * 512, data)
                    self.event.set()
                    
                else:
                    self.log.debug("Received empty packet")
            except:
                time.sleep(0.001)
        
        self.udp_socket.close()
