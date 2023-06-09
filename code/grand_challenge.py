from picam2multithread import Picam2MultiThread
from motordriver import MotorDriver
from ei_monitor import EncoderImuMonitor
from gripperdriver import GripperDriver
from planner import Planner
import time
import cv2
import numpy as np
import RPi.GPIO as gpio

debug = 0
location='umd'
test_arena = 0
 
if location=='home':
    # define the sequence of target block color
    n_repeat = 1
    target_color_sequence = ['red']*n_repeat
    email_receiver = 'liuji1031@gmail.com'
else:
    n_repeat = 3
    target_color_sequence = ['red','green','blue']*n_repeat
    email_receiver = 'ENPM809TS19@gmail.com'
    
if test_arena==1:
    arena_xrange = [-30,170]
    arena_yrange = [-30,170]
    czone_center = [0,120]
    drop_off_offset = 10
    border_offset = 0
else:
    arena_xrange = [-1*30.48,9*30.48]
    arena_yrange = [-1*30.48,9*30.48]
    czone_center = [0.8*30.48,7*30.48]
    drop_off_offset = 8
    border_offset = 15

# initiate and start camera
picam2mt = Picam2MultiThread(main_size=(820,616),lores_size=(205,154),
                             framerate=30.0,verbose=False,
                             rec_vid=1,
                             vid_keepframe_itv=15,
                             vid_framerate=20)
picam2mt.start()
time.sleep(1.0)

# initiate motor driver
# first define PID controller coefficients
Kp_enc,Ki_enc,Kd_enc = [1.0, 0.0, 0.0]
Kp_yaw,Ki_yaw,Kd_yaw = [500.0, 25.0, 0.0]
if location=='umd':
    Kp_turn_pix,Ki_turn_pix,Kd_turn_pix = [0.3, 0.0, 0.4]
else: # home 
    Kp_turn_pix,Ki_turn_pix,Kd_turn_pix = [0.3, 0.0, 0.4]
Kp_turn_rad,Ki_turn_rad,Kd_turn_rad = [80.0, 0.0, 40.0]
Kp_fwd,Ki_fwd,Kd_fwd = [6.0, 0.0, 2.0]
Kp_grab,Ki_grab,Kd_grab = [0.4, 0.0, 0.03]

# create motor driver object
md = MotorDriver(K_enc=(Kp_enc,Ki_enc,Kd_enc),
                 K_yaw=(Kp_yaw,Ki_yaw,Kd_yaw),
                 K_turn_pix=(Kp_turn_pix,Ki_turn_pix,Kd_turn_pix),
                 K_turn_rad=(Kp_turn_rad,Ki_turn_rad,Kd_turn_rad),
                 K_fwd=(Kp_fwd,Ki_fwd,Kd_fwd),
                 K_grab=(Kp_grab,Ki_grab,Kd_grab),
                 speed_sum_max=100,
                 speed_sum_min=30,
                 location=location,
                 arena_xrange=arena_xrange,
                 arena_yrange=arena_yrange,
                 czone_center=czone_center,
                 drop_off_offset=drop_off_offset,
                 target_colors=['red','green','blue','yellow','black'],
                 do_send_email=1,
                 save_planning_img=1,
                 send_planning_email=1,
                 save_position_data=1)

# initiate encoder and imu monitor
monitor = EncoderImuMonitor(log_data=0)
monitor.start()

# initiate planner
planner = Planner(arena_xrange=arena_xrange,
                  arena_yrange=arena_yrange,
                  border_offset=border_offset,
                  k_near=10,
                  max_theta=35)

# initiate gripper
gripper = GripperDriver()

while True:
    start = input('start now?')
    if start=='y':
        print('Let\'s roll!')
        break

for iblock, target_color in enumerate(target_color_sequence):
    while True:
        # get target in view. The following function will look towards
        # some predefined locations. if a target is found, turn towards
        # the coordinate of the block. if not reposition first, and repeat
        # the whole process uptil 5 times
        md.get_target_in_view_v2(target_color, picam2mt, monitor,debug=debug)
        
        #break
        # drive close to target through planning. first it is determined if 
        # target or other blocks are close to robot. If not, drive straight
        # towards target. If close, plan the path accordingly
        success_1 = md.drive_close_to_block(picam2mt, monitor, target_color, planner,
                                            block_id=iblock,debug=debug)
        
        if success_1 == 1:
            print('Driving to target succeeded')
        else:
            print('Driving to target failed, retrying...')
            print('First try repositioning...')
            md.reposition(monitor,reposition_dist=30)
            continue
        
        # pick up target
        success_2 = md.retrieve_target(picam2mt, monitor, target_color, gripper, block_id=iblock)
        if success_2 == 1:
            print('Target retrieved!')
        else:
            print('Retrieving target failed, retrying...')
            continue
        
        # break
        # drive to construction zone, this part should have a very high
        # prob of success
        md.drive_to_czone(picam2mt, monitor, target_color, planner, block_id=iblock, debug=debug)

        # release block
        md.release_block(monitor, gripper)

        # relocalization
        if test_arena==0 and iblock < len(target_color_sequence)-1:
            md.relocalization(monitor)
        break
    
    # now move on to the next block

# finished routine!
md.turn_by_abs_yaw(monitor, -np.pi/4)
gripper.celebrate()

md.save_position_array()

# stop camera
picam2mt.stop()

# stop email thread
md.msgr.stop()

# stop monitor
monitor.stop.value = 1
time.sleep(0.5)
monitor.process.terminate()
monitor.process.join()

# relax gripper
gripper.relax()

# gpio clean up
gpio.cleanup()