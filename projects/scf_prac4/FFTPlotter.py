# -*- coding: utf-8 -*-

import warnings
with warnings.catch_warnings():
    warnings.simplefilter("ignore"); 
    import matplotlib.pyplot as plt

import sys
import threading
import collections
import struct
import logging
import time

from matplotlib import colors
from matplotlib.colors import LogNorm

import numpy as np

class FFTPlotter(threading.Thread):
    def __init__(self, event = None):
        threading.Thread.__init__(self)

        self.event = event

        self.log = logging.getLogger("FFTPlotter")      
        
        self.data = None
        
        self.is_finished = False

    def exit(self):
        self.log.debug("Exitting")
        self.is_finished = True

    def run(self):
        self.log.debug("Starting")

        fig = plt.figure(figsize=(12, 9))

        plt.ion()
        fig.show(False)

        # Two subplots, unpack the axes array immediately
        ax2 = fig.add_subplot(2,1,1)
        ax3 = fig.add_subplot(2,1,2)

        fig.show()
        fig.canvas.draw()

        while not self.is_finished:
            # Wait for image to be received or timeout
            if (self.event.wait(0.001)):
                self.event.clear()

                ax2.clear()
                ax2.set_title("Microphone samples")
                ax2.set_ylabel("Audio Amplitude")
                ax2.set_xlabel("Time Sample #")
                ax2.grid(True)

                ax3.clear()
                ax3.set_title("Microphone FFT")
                ax3.set_ylabel("FFT Magnitude")
                ax3.set_xlabel("Frequency (Hz)")
                ax3.grid(True)
				
                values = np.asarray(self.data, dtype=np.float)
                values = values / np.max(values)
                
                data_fft = abs(np.fft.fft(values)) / len(values)
                data_fft = np.fft.fftshift(data_fft)
                data_fft = data_fft[256:511]

                Fm = 8000.0
                Fft_len = 512.0
                freq_axis = (Fm / Fft_len) * np.arange(0, 255)

                ax2.set_ylim([-1.5,1.5])
                ax2.set_xlim([0,512])
                ax2.plot(values)
                
                ax3.set_ylim([0,1])
                ax3.plot(freq_axis, data_fft)

                fig.canvas.draw()

                fig.canvas.flush_events()

        plt.ioff()
        plt.close()

        self.log.debug("Exitted")
            
    
    def set_data(self, data):
        self.data = data
