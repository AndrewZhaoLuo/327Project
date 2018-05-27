'''
Fairly one-shot throw away!
'''

import math                         # for testing input
import numpy                        # for fast arrays
import serial                       # for reading from serial
import serial.tools.list_ports      # for finding the right port
import time                         # for taking timed pauses
import threading                    # for putting serial read on own thread

import matplotlib.pyplot as plt     # for plotting functions
import seaborn                      # importing automatically makes plotting prettier!

BUFFER_SIZE = 3
SERIAL_LOCK = threading.Lock()

LO_THRESHOLD = 50
HI_THRESHOLD = 80
TRIGGER_SIZE = 500
MAX_TRIGGER_TIME = 150

# dummy serial device that returns a sin wave
# for testing mainly
class DummySerial():
    def __init__(self):
        self.time = 0
        self.rate = 100

    def readline(self):
        v = abs(math.sin(self.time)) * 500

        self.time = (self.time + 0.01) % (2 * math.pi)
        time.sleep(1.0/self.rate)

        return str(v).encode('utf-8')


# opesn a file with data and just repeats the data in there over and over again
class FileSerial():
    def __init__(self, file_name):
        self.time = 0
        self.rate = 100
        self.data = []
        self.index = 0

        f = open(file_name, 'r')
        lines = f.readlines()
        for i, line in enumerate(lines):
            try:
                v = float(line[0:len(line) - 2])
                self.data.append(min(v, 800))

            except ValueError:
                print('Corrupted Data: ', line)

        print('Number of data points: ', len(self.data))

    def readline(self):
        v = self.data[self.index]

        self.index = (self.index + 1) % len(self.data)

        time.sleep(1.0/self.rate)

        return str(v).encode('utf-8')



class ReadSerialThread(threading.Thread):
    def __init__(self, serial, buff):
        super(ReadSerialThread, self).__init__()
        self.serial = serial
        self.buff = buff


    def run(self):
        while True:
            SERIAL_LOCK.acquire()
            serial_read_into_buffer(serial, buff)
            SERIAL_LOCK.release()


class Plotter():
    def __init__(self, buff, MAX_SIZE_BUFFER = 1300, signal_func = lambda f: f, dt = 60 / 1300, smin = 0, smax = 800):
        self.dt = dt        
        self.buff = buff
        self.signal_func = signal_func

        '''
        Buffers for plotting
        '''

        self.data_signal_y = numpy.array([float(0) for i in range(MAX_SIZE_BUFFER)])
        self.data_signal_t = numpy.array([i * dt for i in range(MAX_SIZE_BUFFER)])

        self.data_breaths_y = numpy.array([float(0) for i in range(MAX_SIZE_BUFFER)])
        self.data_breaths_t = numpy.array([float(0) for i in range(MAX_SIZE_BUFFER)])

        self.tick = 0

        self.cur_sum = 0
        self.triggered = False
        self.cur_n = 0
        self.num_breaths = 0
        self.v = 0
        self.time_triggered = 0

        '''
        PLOTS
        '''
        self.figure = plt.figure(figsize=(8, 16))
        self.signal = plt.subplot(211)
        self.signal.set_title('Raw Signal')
        self.signal.set_xlabel('Time(s)')
        self.signal.set_ylabel('Signal (ADC units)')

        self.breaths = plt.subplot(212)
        self.breaths.set_title('Count of breaths')
        self.breaths.set_xlabel('Time(s)')
        self.breaths.set_ylabel('# Breaths')
        self.figure.tight_layout()

        self.signal.set_ylim([smin, smax])
        self.signal.set_xlim([0, dt * MAX_SIZE_BUFFER])
        self.breaths.set_ylim([smin, smax])
        self.breaths.set_xlim([0, dt * MAX_SIZE_BUFFER])

        '''
        Line animations we will update with buffers
        '''
        self.line_signal, = self.signal.plot(self.data_signal_t, self.data_signal_y, 'r-') 
        self.line_breaths, = self.breaths.plot(self.data_breaths_t, self.data_breaths_y, 'r-') 
        self.follower, = self.signal.plot([self.tick, self.tick], [0, smax], 'g-')
        #self.follower_fv, = self.fv.plot([self.tick, self.tick], [fmin, fmax], 'g-')


        self.figure.show()


    def update(self):
        raw_signal = self.signal_func(sum(self.buff) / len(self.buff))
        self.v = raw_signal * 0.0549 + self.v * 0.945 
        signal = self.v
        #print(signal)

        if self.tick == 0:
            print('Start')
            print('Num breaths: ', self.num_breaths)
            self.data_signal_y.fill(0.0)
            self.data_signal_t.fill(0.0)
            self.data_breaths_y.fill(0.0)
            self.data_breaths_t.fill(0.0)

            self.cur_sum = 0
            self.triggered = False
            self.cur_n = 0
            self.num_breaths = 0
            self.time_triggered = 0

        else:
            self.data_signal_y[self.tick] = signal
            self.data_signal_t[self.tick] = self.data_signal_t[self.tick - 1] + 1 * self.dt

            '''
            Schmitt Trigger processing
            '''
            v = 0
            if not self.triggered:
                self.cur_n += 1

                if signal >= HI_THRESHOLD + self.cur_sum / self.cur_n:
                    v = self.cur_sum / self.cur_n + TRIGGER_SIZE
                    self.num_breaths += 1
                    self.triggered = True
                else:
                    v = self.cur_sum / self.cur_n

                self.cur_sum += signal
                self.time_triggered = 0

            if self.triggered:
                v = self.cur_sum / self.cur_n + TRIGGER_SIZE
                
                if signal <= LO_THRESHOLD + self.cur_sum / self.cur_n:
                    self.triggered = False 
                
                self.time_triggered += 1
                if self.time_triggered > MAX_TRIGGER_TIME:
                    self.triggered = False
                    self.cur_sum = signal
                    self.cur_n = 1
                    self.time_triggered = 0
                    

            self.data_breaths_y[self.tick] = v
            self.data_breaths_t[self.tick] = self.data_signal_t[self.tick - 1] + 1 * self.dt
                

        self.tick = (self.tick + 1) % len(self.data_signal_y)
    
        # updating raw signal graph
        self.line_signal.set_ydata(self.data_signal_y[0:self.tick])
        self.line_signal.set_xdata(self.data_signal_t[0:self.tick])

        # updating breaths graph
        self.line_breaths.set_ydata(self.data_breaths_y[0:self.tick])
        self.line_breaths.set_xdata(self.data_breaths_t[0:self.tick])

        # updating cool follower
        self.follower.set_xdata([self.tick * self.dt, self.tick * self.dt])
        self.follower.set_ydata([0, 1000])

        self.figure.canvas.draw()


# Initializes the class which reads from the arduino
def init_serial(baud_rate, device_name, timeout=5):
    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        print (port)
        if device_name in str(port):
            return serial.Serial(port.device, baud_rate, timeout=timeout)
    
    print('ERROR: could not find port of device with name', device_name)
    return None


# Takes a serial class, and async. write to a buffer
def serial_read_into_buffer(serial, buff):
    cur_line = serial.readline()
    
    # once in a while a point fails to send it seems
    # like the message is random bits
    try:
        cur_line = float(cur_line.decode('utf-8').strip())
    except UnicodeDecodeError:
        return
    except ValueError:
        return

    value = cur_line

    buff[0:-1] = buff[1:]
    buff[-1] = value


if __name__ == "__main__":
    '''
    For the buffer, it's crucial to numpy arrays!!
    Python arrays --> [int*, int*, int* ...]
     Numpy arrays --> [int,  int,  int  ...]

    Numpy speed >> Python speeds
    Because there is no pointer redirection nonsense -- better l1/l2 util.
    '''
    buff = numpy.array([0.0] * BUFFER_SIZE)
    serial = FileSerial('haley_7_times_60s_deep_breahts_still.txt')#init_serial(9600, 'rfcomm')
    plotter = Plotter(buff)

    #serial =  DummySerial()
    #plotter = Plotter(buff, fmax = 1, fmin = -1, vmax = 10)

    reading_thread = ReadSerialThread(serial, buff)
    reading_thread.start()

    while True:
        plotter.update()
    
    
