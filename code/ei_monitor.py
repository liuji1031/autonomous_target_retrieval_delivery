from multiprocessing import Value, Process
import time
import numpy as np
import RPi.GPIO as gpio
import serial
from datetime import datetime

class EncoderImuMonitor:
    def __init__(self,log_data=0):
        # encoder pins
        self.left_pin = 7
        self.right_pin = 12
        self.pulse_per_rev = 480 # remember the gear ratio
        
        # set up multiprocessor variables
        self.left_count = Value('l',0) # 'l': long integer
        self.right_count = Value('l',0)
        self.setup_complete = Value('i',0) # 'i': integer
        self.stop = Value('i',0) # 'i': integer
        self.start_logging = Value('i',0)
        self.max_count = -1
        self.log_data = log_data
        
        self.imu_reading = Value('d',0) # 'd': double precision float
        self.imu_setup_complete = Value('i',0)
        self.imu_stop = Value('i',0)
        self.imu_read_count = 0
        self.imu_curr_read = 0.
        
        # start monitoring encoder and imu
        self.process = Process(target=self.monitor_encoder_imu,
                    args=(self.left_count,
                          self.right_count,
                          self.stop,
                          self.max_count,
                          self.log_data,
                          self.start_logging,
                          self.imu_reading,
                          self.setup_complete,))

    def start(self):
        self.process.start()
        print('starting',end='',flush=True)
        while self.setup_complete.value==0: # or self.imu_setup_complete.value==0:
            print('.',end='',flush=True)
            time.sleep(0.1)
    
    def stop(self):
        # clean up
        self.stop.value = 1
        time.sleep(0.5)
        self.process.terminate()
        self.process.join()
        
    def read_imu(self,ser):
        if(ser.in_waiting > 0):
            self.imu_read_count+=1
            if self.imu_read_count<10:
                return None
            line = str(ser.readline().rstrip().lstrip())
            line = line.strip("'")
            line = line.strip("b'")
            try:
                reading = -float(line)/180*np.pi
                return reading
            except:
                return None
        else:
            return None
    
    def monitor_encoder_imu(self,left_count,right_count,stop,max_count=-1,
                         log_data=0,start_logging=1,imu_reading=0.,
                         setup_complete=0):
        # start imu port
        try:
            ser = serial.Serial('/dev/ttyUSB0',9600)
        except:
            ser = serial.Serial('/dev/ttyUSB1',9600) 
        # flush
        while True:
            imu_test_read = self.read_imu(ser)
            if imu_test_read is not None:
                imu_reading.value = imu_test_read
                print(imu_test_read)
                break
        print("imu port started")
        
        # set up encoder pins
        gpio.setmode(gpio.BOARD)
        gpio.setup(self.left_pin,gpio.IN,pull_up_down=gpio.PUD_UP)
        gpio.setup(self.right_pin,gpio.IN,pull_up_down=gpio.PUD_UP)
        print("encoder counter started, log_data = ",log_data)
        setup_complete.value = 1
        
        # create array to store data
        if log_data == 1:
            data = np.zeros((70000,3))
            timestamp_left = []
            timestamp_right = []
            idata=0

        # set initial value
        MA_win = 20
        iloop = 0
        start_time = time.time_ns()
        last_time = start_time
        left_buffer = []
        right_buffer = []
        while stop.value == 0:
            # keep track of loop time
            iloop+=1
            
            # read imu
            imu_test_read = self.read_imu(ser)
            if imu_test_read is not None:
                imu_reading.value = imu_test_read

            # read encoder
            curr_left_pin = int(gpio.input(self.left_pin))
            curr_right_pin = int(gpio.input(self.right_pin))
            left_buffer.append(curr_left_pin)
            right_buffer.append(curr_right_pin)
            
            # moving window average
            if iloop<=MA_win:
                curr_left = np.mean(left_buffer)
                curr_right = np.mean(right_buffer)
                last_left = curr_left
                last_right = curr_right
                continue
            else:
                curr_left = curr_left + (curr_left_pin - left_buffer.pop(0))/MA_win
                curr_right = curr_right + (curr_right_pin - right_buffer.pop(0))/MA_win
                
            # log encoder data if needed
            curr_time = time.time_ns()
            if log_data == 1:
                if curr_time - last_time >= 0.0005*1e9 and start_logging.value==1: # record data every 1 ms
                    last_time = curr_time
                    if idata<data.shape[0]:
                        data[idata,0]=curr_left
                        data[idata,1]=curr_right
                        data[idata,2]=curr_time
                        idata+=1

            th1 = 0.9
            th2 = 0.1
            if (curr_left>th1 and last_left<=th1) or (curr_left<th2 and last_left>=th2):
                left_count.value += 1
                if log_data==1:
                    timestamp_left.append(curr_time)
            last_left = curr_left

            if (curr_right>th1 and last_right<=th1) or (curr_right<th2 and last_right>=th2):
                right_count.value += 1
                if log_data==1:
                    timestamp_right.append(curr_time)
            last_right = curr_right

        end_time = time.time_ns()
        loop_time = (end_time-start_time)/1e9/iloop
        print("process stopped, left count,",left_count.value,"right count",right_count.value,"avg loop time",loop_time)
        
        # save data
        if log_data==1:
            data = data[:idata,:]
            data = np.vstack((data,np.array([[left_count.value,right_count.value,0]])))
            #plt.plot(data)
            now = datetime.now() # current date and time
            date_time = now.strftime("%Y-%m-%d_%H-%M-%S")
            fn = 'encoder_data_both_'+date_time+'.txt'
            np.savetxt(fn,data)
            fn_l = 'TS_left_'+date_time+'.txt'
            ts_l = np.array(timestamp_left)
            np.savetxt(fn_l,ts_l)
            fn_r = 'TS_right_'+date_time+'.txt'
            ts_r = np.array(timestamp_right)
            np.savetxt(fn_r,ts_r)
            print('data saved',fn)
            
    def monitor_imu(self,imu_reading=0.,setup_complete=0,stop=0):
        # start imu port
        ser = serial.Serial('/dev/ttyUSB0',9600)
        # flush
        while True:
            imu_test_read = self.read_imu(ser)
            if imu_test_read is not None:
                imu_reading.value = imu_test_read
                print(imu_test_read)
                break
        print("imu port started")
        
        setup_complete.value = 1
        
        # set initial value
        iloop = 0

        start_time = time.time_ns()
        while stop.value == 0:
            # keep track of loop time
            iloop+=1
            
            # read imu
            imu_test_read = self.read_imu(ser)
            if imu_test_read is not None:
                imu_reading.value = imu_test_read
                # self.imu_curr_read = imu_test_read
                # print('----',imu_test_read/np.pi*180)
        end_time = time.time_ns()
        loop_time = (end_time-start_time)/1e9/iloop
        print("process stopped, avg loop time",loop_time)
