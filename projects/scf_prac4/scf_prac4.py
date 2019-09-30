# -*- coding: utf-8 -*-

import warnings
with warnings.catch_warnings():
    warnings.simplefilter("ignore"); 
    import matplotlib.pyplot as plt

import sys
import signal
import threading
import time
import logging
import math
import collections
import numpy as np

import UDPServer
import FFTPlotter

is_finished = False

def signal_handler(signal, frame):
    global is_finished
    print "You pressed CTRL + C!"
    is_finished = True

def main():
    global is_finished

    # Get logger class
    log = logging.getLogger("Main")

    # Create signal handler event
    signal.signal(signal.SIGINT, signal_handler)

    # Create asynchronous events
    dataEvent = threading.Event()
    plotEvent = threading.Event()

    # Clear events
    dataEvent.clear()
    plotEvent.clear()

    # Create threads
    udp_server = UDPServer.UDPServer(event = dataEvent, host = "0.0.0.0", udp_port = 8080)
    fft_plotter = FFTPlotter.FFTPlotter(event = plotEvent)
    
    # Start threads
    udp_server.start()
    fft_plotter.start()
    
    print "Press CTRL+C to stop!"

    log.debug("Starting...")     
			
    # Until program is not finished
    while (not is_finished):
        if (dataEvent.wait(0.01)):
            # Clear image event
            dataEvent.clear()

            last_data = udp_server.get_last_packet()

            fft_plotter.set_data(last_data)
            plotEvent.set()

    log.debug("Finishing...")

    fft_plotter.exit()
    udp_server.exit()

    fft_plotter.join()
    udp_server.join()

    log.debug("Finished")

if __name__ == '__main__':
    logging.basicConfig(level=logging.NOTSET,
                        filename='scf_prac4.log', filemode='w+',
                        format='%(name)s %(levelname)s %(message)s')

    main() 
