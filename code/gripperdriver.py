import pigpio
import time
import numpy as np

class GripperDriver():
    def __init__(self):
        self.lower_bound = 5.5
        self.upper_bound = 9.0
        self.pwm = pigpio.pi()
        self.pin_number = 16 # use GPIO number
        self.pwm.set_mode(self.pin_number,pigpio.OUTPUT)
        self.freq = 50
        self.period_us = 1000000./self.freq
        self.pwm.set_PWM_frequency(self.pin_number, self.freq)
        
        self.close_gripper()
    
    def relax(self):
        self.pwm.set_PWM_dutycycle(self.pin_number,0)
        self.pwm.set_PWM_frequency(self.pin_number,0)
        
    def command(self,PWM):
        PWM = min(PWM, self.upper_bound)
        PWM = max(PWM, self.lower_bound)
        self.pwm.set_servo_pulsewidth(self.pin_number,
                                      PWM/100*self.period_us)
    def close_gripper(self):
        PWM = self.lower_bound
        self.pwm.set_servo_pulsewidth(self.pin_number,
                                      PWM/100*self.period_us)
    def open_gripper(self):
        PWM = self.upper_bound
        self.pwm.set_servo_pulsewidth(self.pin_number,
                                      PWM/100*self.period_us)
        
    def test(self):
        for i in range(3):
            self.command(5.0)
            time.sleep(3.0)
            self.command(7.0)
            time.sleep(3.0)
        gd.cleanup()

    def celebrate(self, n=2):
        for i in range(n):
            self.close_gripper()
            time.sleep(0.3)
            self.open_gripper()
            time.sleep(0.3)
        self.close_gripper()