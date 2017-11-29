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

# dummy serial device that returns a sin wave
# for testing mainly
class DummySerial():
    def __init__(self):
        self.time = 0
        self.rate = 100

    def readline(self):
        v = math.sin(self.time)

        self.time = (self.time + 0.01) % (2 * math.pi)
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
    def __init__(self, buff, MAX_SIZE_BUFFER = 300, flow_func = lambda f: f / 2830.350827,
                       dt = 1, fmax = 300000, fmin = -150000, vmax = 40000):
        self.dt = dt        
        self.buff = buff
        self.flow_func = flow_func

        self.data_vt_v = numpy.array([float(0) for i in range(MAX_SIZE_BUFFER)])
        self.data_vt_t = numpy.array([i * dt for i in range(MAX_SIZE_BUFFER)])

        self.data_fv_f = numpy.array([float(0) for i in range(MAX_SIZE_BUFFER)])
        self.data_fv_v = numpy.array([float(0) for i in range(MAX_SIZE_BUFFER)])

        self.figure = plt.figure(figsize=(8, 16))
        self.vt = plt.subplot(211)
        self.vt.set_title('Volume vs. Time')
        self.vt.set_xlabel('Time(s)')
        self.vt.set_ylabel('Volume Exhaled (L)')

        self.fv = plt.subplot(212)
        self.fv.set_title('Flow vs. Volume')
        self.fv.set_xlabel('Volume Exhaled (L)')
        self.fv.set_ylabel('Flow (L / s)')
        self.figure.tight_layout()

        self.vt.set_ylim([0, vmax])
        self.fv.set_ylim([fmin, fmax])
        self.fv.set_xlim([0, vmax])

        self.line_vt, = self.vt.plot(self.data_vt_t, self.data_vt_v, 'r-') 
        self.line_fv, = self.fv.plot(self.data_fv_v, self.data_fv_f, 'r-') 

        self.tick = 0

        self.follower_vt, = self.vt.plot([self.tick, self.tick], [0, vmax], 'g-')
        #self.follower_fv, = self.fv.plot([self.tick, self.tick], [fmin, fmax], 'g-')

        self.figure.show()


    def update(self):
        cur_f = self.flow_func(sum(self.buff) / len(self.buff))

        if self.tick == 0:
            print('FVC (L):', max(self.data_vt_v))

            # calculate FEV1
            start = -10000
            for i in range(len(self.data_vt_v)):
                if self.data_vt_v[i] > 0.1:
                    start = i
                    break
            FEV_i = min(round(start + 1 / self.dt), len(self.data_vt_v) - 1)
            if FEV_i > 0:
                print('FEV1 (L):', self.data_vt_v[FEV_i])
                print('FEV1/FVC:', self.data_vt_v[FEV_i] / max(self.data_vt_v))
            else:
                print('FEV1 (L) could not be estimated!')
            print()

            self.data_vt_v.fill(0.0)
            self.data_fv_f.fill(0.0)
            self.data_fv_v.fill(0.0)
        else:
            self.data_fv_f[self.tick] = cur_f
            # don't count inspiration toward volume
            #if cur_f < 0:
            #    cur_f = 0
            dV = cur_f * 1.0 * self.dt
            self.data_vt_v[self.tick] = self.data_vt_v[self.tick - 1] + dV

            self.data_fv_v[self.tick] = self.data_vt_v[self.tick]

        self.tick = (self.tick + 1) % len(self.data_vt_v)


        # only plot increases in volume for vt graph
        data_fv_v_positive = numpy.copy(self.data_vt_v)
        for i in range(1, self.tick + 1):
            data_fv_v_positive[i] = max(data_fv_v_positive[i - 1], data_fv_v_positive[i])
        self.line_vt.set_ydata(data_fv_v_positive)

        self.line_fv.set_ydata(self.data_fv_f[0:self.tick])
        self.line_fv.set_xdata(self.data_fv_v[0:self.tick])

        self.follower_vt.set_xdata([self.data_vt_t[self.tick], self.data_vt_t[self.tick]])
        #self.follower_fv.set_xdata([self.data_fv_v[self.tick], self.data_fv_v[self.tick]])

        self.figure.canvas.draw()


# Initializes the class which reads from the arduino
def init_serial(baud_rate, device_name, timeout=5):
    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        if device_name in port.description:
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
    serial = init_serial(9600, 'Arduino')
    plotter = Plotter(buff, flow_func = lambda f: f / 2830.350827, dt = 1 / 16.67, fmax = 15, fmin = -3, vmax = 8)

    #serial =  DummySerial()
    #plotter = Plotter(buff, fmax = 1, fmin = -1, vmax = 10)

    reading_thread = ReadSerialThread(serial, buff)
    reading_thread.start()

    while True:
        plotter.update()
    
    
