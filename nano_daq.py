import threading
import serial
from datetime import datetime
import csv
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SerialPlotter:
    def __init__(self, port, baudrate=9600, max_len=500, plot_interval=1, csv_filename=None):
        self.ser = serial.Serial(port=port, baudrate=baudrate)
        self.max_len = max_len
        self.plot_interval = plot_interval
        self.csv_filename = csv_filename

        self._animation = None 

        # Initialize containers
        self.timestamps = deque([], maxlen=self.max_len)
        self.adc = deque([], maxlen=self.max_len)
        self._last_timestamp = 0
        self._last_adc = 0

        # Initialize plots
        self._fig, self._ax = plt.subplots()
        self._line, = self._ax.plot(self.timestamps, self.adc)
        self._ax.set_title('ADC')
        self._ax.set_xlabel('Time (s)')
        self._ax.set_ylabel('ADC (a. u.)')
        
        # Start background thread for reading data from serial port
        self._serial_thread = threading.Thread(target=self._read_serial)
        self._stop_event = threading.Event()
        
        
    def start(self):
        # background thread
        self._serial_thread.start()

        # start animation
        self._animation = FuncAnimation(self._fig, self._update_plots, interval=self.plot_interval, cache_frame_data=False)
        
    def stop(self):
        self._stop_event.set()
        self._serial_thread.join()
    
    def _read_serial(self):
        while not self._stop_event.is_set():
        # Read data from serial port
            try:
                serial_byte = self.ser.readline()
                serial_line = serial_byte.decode('utf-8').strip()
                serial_split = serial_line.split('\t')
            except UnicodeDecodeError:
                continue
            except IndexError:
                continue
            
        
            if len(serial_split) != 2: # incomplete line
                continue
            timestamp_string = serial_line.split('\t')[0]
            photovoltage_string = serial_line.split('\t')[1]

            self._last_timestamp = int(timestamp_string)/1000
            self._last_adc = int(photovoltage_string)
            self.timestamps.append(self._last_timestamp)
            self.adc.append(self._last_adc)

            if self.csv_filename:
                with open(self.csv_filename, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([self._last_timestamp, self._last_adc])
            
    def _update_plots(self, frame):
        # Update plot data
        self._line.set_xdata(self.timestamps)
        self._line.set_ydata(self.adc)
        self._ax.relim()
        self._ax.autoscale_view()
        
        # Set line color
        self._line.set_color('#3a79cf')
        
        return self._line
        
if __name__ == '__main__':
     # Find the correct serial port for your device
    import serial.tools.list_ports
    
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        print(port)

    # Replace the serial port with the correct one for your device
    serial_port = input("Enter the serial port for your device: ")
    
    now = datetime.now()
    year = now.strftime('%Y')
    month = now.strftime('%m')
    day = now.strftime('%d')
    hour = now.strftime('%H')
    minute = now.strftime('%M')
    second = now.strftime('%S')
    filename = '-'.join([year, month, day, hour, minute, second])

    serial_plotter = SerialPlotter(serial_port, csv_filename=filename+'.txt')
    serial_plotter.start()
    plt.show(block=True)

    input('Press Enter to stop...\n')
    serial_plotter.stop()