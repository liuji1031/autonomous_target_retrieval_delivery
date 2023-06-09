import RPi.GPIO as gpio
import time
import matplotlib.pyplot as plt
import numpy as np

class DistanceSensor:
    def __init__(self,n_repeat=10):
        # define pin allocation
        self.trig = 16
        self.echo = 18
        self.echo_timeout = 0.2
        self.speed = 34000.0 # speed of sound cm / sec
        self.max_dist = 10.0 # assume a max distance of 10 meter
        self.max_dur = 2*self.max_dist*100.0/self.speed
        self.n_repeat = n_repeat # each time a measurement is requested, do several repeats and report median
        
        gpio.setmode(gpio.BOARD)
        gpio.setup(self.trig,gpio.OUT)
        gpio.setup(self.echo,gpio.IN)

    def measure(self):
        v_last = 0
        t_start = -1
        t_end = -1
        
        # ensure output has no value
        gpio.output(self.trig,False)
        time.sleep(1e-6)
        
        # generate trigger pulses
        gpio.output(self.trig,True)
        time.sleep(0.00001)
        gpio.output(self.trig,False)
        
        t_begin = time.time_ns()
        while True:
            v = gpio.input(self.echo)
            # signal[i] = v
            # print((time.time_ns()-t_start)/1e9)
            t_curr = time.time_ns()
            if v_last==0 and v==1:
                t_start = t_curr
            if v_last==1 and v==0:
                t_end = t_curr
            v_last = v
            
            if (t_curr-t_begin)/1e9>self.max_dur or t_end!=-1:
                break

        if t_start==-1 or t_end==-1:
            # print("time out!")
            return -1.0
        t_start-=t_begin
        t_end-=t_begin
        pulse_duration = (t_end-t_start)/1e9
        distance = round(self.speed * pulse_duration / 2.0,2)
        # print(pulse_duration,distance)
        return distance
        """
        # generate echo time signal
        pulse_start = time.time_ns()
        while gpio.input(self.echo) == 0:
            pulse_start = time.time_ns()
        
        while gpio.input(self.echo) == 1:
            pulse_end = time.time_ns()
            if float(pulse_end - pulse_start)/1e9 >= self.echo_timeout:
                distance = -1.0
                return distance
            
        pulse_duration = float(pulse_end - pulse_start)/1e9
        
        distance = round(speed * pulse_duration / 2.0,2)
        self.current_distance = distance
        
        return distance
        """
    
    def stop(self):
        # cleanup gpio pins
        gpio.cleanup()
    
    def get_distance(self):
        measurement = []
        for i in range(self.n_repeat):
            m = self.measure()
            if m>0:
                measurement.append(m)
        # gpio.cleanup()
        measurement_array = np.array(measurement)
        if len(measurement) > self.n_repeat/2:
            return np.median(measurement_array), measurement_array
        else:
            return -1, measurement_array