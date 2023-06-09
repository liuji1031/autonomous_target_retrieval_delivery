import numpy as np
import RPi.GPIO as gpio
import matplotlib.pyplot as plt
from threading import Thread
from multiprocessing import Process, Value
import time
from distancesensor import DistanceSensor
import serial
import cv2
from datetime import datetime
from ei_monitor import EncoderImuMonitor
# from messenger import Messenger
from messengermultithread  import Messenger

class MotorDriver:
    
    def __init__(self,
                 K_enc=[0,0,0],
                 K_yaw=[0,0,0],
                 K_turn_pix=[0,0,0],
                 K_turn_rad=[0,0,0],
                 K_fwd=[0,0,0],
                 K_grab=[0,0,0],
                 speed_sum_max=80,
                 speed_sum_min=0,
                 location='umd',
                 log_data=0,
                 email_receiver='liuji1031@yahoo.com',
                 arena_xrange=[-1*30.48,9*30.48],
                 arena_yrange=[-1*30.48,9*30.48],
                 czone_center=[1*30.48,7*30.48],
                 drop_off_offset=15,
                 target_colors=['red','green','blue','yellow','black'],
                 pause_time=0.25,
                 do_send_email=0,
                 save_planning_img=1,
                 send_planning_email=0,
                 save_position_data=0):
        freq = 100
        gpio.setmode(gpio.BOARD)
        gpio.setup(31,gpio.OUT) # IN1
        gpio.setup(33,gpio.OUT) # IN2
        gpio.setup(35,gpio.OUT) # IN3
        gpio.setup(37,gpio.OUT) # IN4
        
        self.pwm1 = gpio.PWM(31,freq)
        self.pwm2 = gpio.PWM(33,freq)
        self.pwm3 = gpio.PWM(35,freq)
        self.pwm4 = gpio.PWM(37,freq)
        
        self.pwm1.start(0.)
        self.pwm2.start(0.)
        self.pwm3.start(0.)
        self.pwm4.start(0.)
        
        self.left_pin = 7
        self.right_pin = 12
        # self.pulse_per_rev = 480 # remember the gear ratio

        gpio.setup(self.left_pin,gpio.IN,pull_up_down=gpio.PUD_UP)
        gpio.setup(self.right_pin,gpio.IN,pull_up_down=gpio.PUD_UP)
        
        self.dist_sensor = DistanceSensor()
        
        self.speed_sum_max = speed_sum_max
        self.speed_sum_min = speed_sum_min
        self.controller_hz = 20
        self.ctrl_loop_dur = 1./self.controller_hz
        self.count_per_rev = 40
        self.wheel_d = 6.5
        
        self.Kp_enc,self.Ki_enc,self.Kd_enc = K_enc
        self.Kp_yaw,self.Ki_yaw,self.Kd_yaw = K_yaw
        self.Kp_turn_pix,self.Ki_turn_pix,self.Kd_turn_pix = K_turn_pix
        
        self.max_turn_speed = 40
        self.desired_center_x = int((413+19)/4)
        self.desired_center_y = 145
        self.tol_pixel = 7
        self.tol_pixel_coarse = 15 # 1/3 of field of view
        
        self.tol_rad = 1/180*np.pi
        
        self.location = location

        if self.location =='home':
            # home
            self.min_turn_pwm = [32,38]
            self.max_turn_pwm = [40,48]
        else:
            # umd
            self.min_turn_pwm = [32,35]
            self.max_turn_pwm = [40,48]
        
        self.Kp_turn_rad,self.Ki_turn_rad,self.Kd_turn_rad = K_turn_rad
    
        self.Kp_fwd,self.Ki_fwd,self.Kd_fwd = K_fwd
        self.tol_dist = 2
        
        self.Kp_grab,self.Ki_grab,self.Kd_grab = K_grab

        self.curr_x = 0
        self.curr_y = 0
        self.save_position_data = save_position_data
        if self.save_position_data == 1:
            self.pos_array = []
            self.append_curr_pos()

        # homography for mapping pixel to world point
        self.H = np.array([[-0.0062,0.1869,-117.4956],[0.1428,-0.0089,-59.5014],[0.0001,-0.0104,1.]])
        self.H_inv = np.linalg.inv(self.H)

        self.size_threshold = 0.00015
        self.cam_base_xoffset = 20

        self.do_send_email = do_send_email
        self.msgr = Messenger(receiver=email_receiver)
        if self.do_send_email==1:
            self.msgr.start()

        self.arena_xrange = arena_xrange
        self.arena_yrange = arena_yrange
        # define construction zone center
        self.czone_center_worldfr = np.array(czone_center).reshape((1,2)) 
        self.drop_off_offset = drop_off_offset # the offset for dropping each block

        self.target_colors = target_colors
        self.pause_time = pause_time

        self.send_planning_email = send_planning_email
        self.save_planning_img = save_planning_img

        # define a set of locations for dropping the target
        y,x = np.meshgrid(np.arange(1,-2,-1),np.arange(-1,2,1))
        self.drop_loc = self.czone_center_worldfr + np.array([[x_,y_] for (x_,y_) in zip(x.flatten(),y.flatten())])*self.drop_off_offset

    def append_curr_pos(self):
        self.pos_array.append([time.time(), self.curr_x, self.curr_y])

    def gameover(self):
        # set all pins low      
        self.pwm1.ChangeDutyCycle(0.)
        self.pwm2.ChangeDutyCycle(0.)
        self.pwm3.ChangeDutyCycle(0.)
        self.pwm4.ChangeDutyCycle(0.)
    
    def set_speed_left(self,speed=100.0):
        speed_abs = abs(speed)
        speed_abs = min(speed_abs,100.)
        speed_abs = max(speed_abs,0.)
        if speed>0:
            self.pwm1.ChangeDutyCycle(speed_abs)
            self.pwm2.ChangeDutyCycle(0.)
        else:
            self.pwm1.ChangeDutyCycle(0.)
            self.pwm2.ChangeDutyCycle(speed_abs)
    
    def set_speed_right(self,speed=100.0):     
        speed_abs = abs(speed)
        speed_abs = min(speed_abs,100.)
        speed_abs = max(speed_abs,0.)
        if speed>0:
            self.pwm3.ChangeDutyCycle(0.)
            self.pwm4.ChangeDutyCycle(speed_abs)
        else:
            self.pwm3.ChangeDutyCycle(speed_abs)
            self.pwm4.ChangeDutyCycle(0.)
    
    def calculate_rot_matrix_from_rad(self,rad):
        c = np.cos(rad)
        s = np.sin(rad)
        rot_mat = np.array([[c,-s],[s,c]])
        return rot_mat
        
    def convert_rad_in_ref(self,rot_mat, yaw):
        yaw_vec = np.array([[np.cos(yaw)],[np.sin(yaw)]])
        yaw_vec_ref = rot_mat.T.dot(yaw_vec)
        rad_in_ref = np.arctan2(yaw_vec_ref[1],yaw_vec_ref[0]).item()
        return rad_in_ref
    
    def clip_speed(self,speed):
        speed = min(self.speed_sum_max,speed)
        speed = max(self.speed_sum_min,speed)
        return speed
    
    def update_odom(self,yaw,delta_dist,sign):
        dx = np.cos(yaw)
        dy = np.sin(yaw)
        self.curr_x += sign*delta_dist*dx
        self.curr_y += sign*delta_dist*dy

    def calculate_distance_traveled(self, left_count, right_count):
        return (left_count+right_count)/2.0/self.count_per_rev*np.pi*self.wheel_d

    def report_current_pose(self,monitor):
        yaw = monitor.imu_reading.value
        yaw_degree = yaw/np.pi*180
        if yaw_degree<0:
            yaw_degree+=360
        print('Current location ({:.2f},{:.2f}), heading angle {:.2f}'.format(self.curr_x,self.curr_y,yaw_degree))

    def update_curr_pos(self, LC_prev, LC, RC_prev, RC, yaw, sign=1):
        dist = self.calculate_distance_traveled(LC-LC_prev,RC-RC_prev)
        self.update_odom(yaw, dist, sign)
        if self.save_position_data == 1:
            self.append_curr_pos()
    
    def save_position_array(self):
        if self.save_position_data==1:
            now = datetime.now() # current date and time
            date_time = now.strftime("%Y-%m-%d_%H-%M-%S")
            fn = 'Position_Data_'+date_time+'.txt'
            np.savetxt(fn, np.array(self.pos_array))

    def forward_controller(self,max_dist,backwards=False,monitor=None,log_data=0):
        
        if backwards == True:
            speed_multiplier = -1
        else:
            speed_multiplier = 1
               
        # initialize variables
        start_count_left = monitor.left_count.value
        start_count_right = monitor.right_count.value
        print('start count',start_count_left,start_count_right)
        start_yaw = monitor.imu_reading.value
        rot_mat = self.calculate_rot_matrix_from_rad(start_yaw)

        err_dist = max_dist
        err_dist_sum = 0.
        err_dist_diff = 0.
        err_dist_last = err_dist
        
        err_enc_sum = 0.
        err_enc_diff = 0.
        err_enc_last = 0.
        
        err_yaw_sum = 0.
        err_yaw_diff = 0.
        err_yaw_last = 0.
        
        # set initial speed
        speed_sum = self.Kp_fwd*err_dist+self.Ki_fwd*err_dist_sum+self.Kd_fwd*err_dist_diff
        speed_sum = self.clip_speed(speed_sum)
        speedL = speed_sum / 2.
        speedR = speed_sum / 2.
        self.set_speed_left(speed_multiplier*speedL)
        self.set_speed_right(speed_multiplier*speedR)

        last_time = time.time_ns()
        log_dt = 0.02
        last_time_log = last_time
        if log_data==1:
            data = np.zeros((50*30,3)) # assume 20 hz controller, max dur 30 sec
            idata = 0

        iter = 0
        avg_yaw_in_ref = 0
        while True:
            iter+=1
            curr_time = time.time_ns()
            loop_time = float(curr_time-last_time) / 1e9
            if loop_time < self.ctrl_loop_dur:
                time.sleep(0.0001)
                continue
            else:
                last_time = curr_time
            
            # track imu
            curr_yaw = monitor.imu_reading.value
            curr_yaw_in_ref = self.convert_rad_in_ref(rot_mat,curr_yaw)
            avg_yaw_in_ref = (avg_yaw_in_ref*(iter-1) + curr_yaw_in_ref)/iter
            if backwards==False:
                err_yaw = -curr_yaw_in_ref
            else:
                err_yaw = curr_yaw_in_ref
            err_yaw_sum = err_yaw_sum + err_yaw
            err_yaw_diff = err_yaw - err_yaw_last
            err_yaw_last = err_yaw
            
            # track encoder
            LC = monitor.left_count.value-start_count_left
            RC = monitor.right_count.value-start_count_right
            # track encoder error
            err_enc = LC-RC
            err_enc_sum = err_enc_sum + err_enc
            err_enc_diff = err_enc - err_enc_last
            err_enc_last = err_enc
            # print(LC,RC)
            # track distance
            dist = self.calculate_distance_traveled(LC,RC)

            if log_data==1 and idata<data.shape[0]:
                data[idata,0] = curr_time
                data[idata,1] = dist
                data[idata,2] = curr_yaw
                idata += 1
                
            if abs(dist-max_dist)<self.tol_dist:
                break
            # compute distance error and speed sum
            err_dist = max_dist-dist
            err_dist_sum += err_dist
            err_dist_diff = err_dist - err_dist_last
            err_dist_last = err_dist
            speed_sum = self.Kp_fwd*err_dist+self.Ki_fwd*err_dist_sum+self.Kd_fwd*err_dist_diff
            speed_sum = self.clip_speed(speed_sum)
            
            # compute speed difference between wheel
            speed_diff = self.Kp_enc*err_enc + self.Ki_enc*err_enc_sum + self.Kd_enc*err_enc_diff + \
                         self.Kp_yaw*err_yaw + self.Ki_yaw*err_yaw_sum + self.Kd_yaw*err_yaw_diff
            speedR = (speed_sum + speed_diff)/2
            speedL = speed_sum - speedR
            
            speedL = speed_multiplier*speedL
            speedR = speed_multiplier*speedR
            
            if speed_multiplier>0:
                speedL = max(0,speedL)
                speedR = max(0,speedR)
            else:
                speedL = min(0,speedL)
                speedR = min(0,speedR)
            
            # command speed
            self.set_speed_left(speedL)
            self.set_speed_right(speedR)
            
            # print(speed_sum,LC,RC,err_enc,np.round(err_yaw,3),np.round(speedL,1),np.round(speedR,1))
            
        # stop 
        self.set_speed_left(0) 
        self.set_speed_right(0)
        time.sleep(self.pause_time)
        end_count_left = monitor.left_count.value
        end_count_right = monitor.right_count.value
        # print('end count',end_count_left,end_count_right)

        # track odom one more time
        if backwards==True:
            sign = -1
        else:
            sign = 1
        self.update_curr_pos(start_count_left, end_count_left, 
                            start_count_right, end_count_right,
                            start_yaw+avg_yaw_in_ref, 
                            sign)
        self.report_current_pose(monitor)
        
        # save data one more time
        if log_data==1 and idata<data.shape[0]:
            curr_time = time.time_ns()
            LC = end_count_left-start_count_left
            RC = end_count_right-start_count_right
            dist = (LC+RC)/2.0/self.count_per_rev*np.pi*self.wheel_d
            curr_yaw = monitor.imu_reading.value

            data[idata,0] = curr_time
            data[idata,1] = dist
            data[idata,2] = curr_yaw
            
            # save data
            data = data[:idata+1,:]
            
            now = datetime.now() # current date and time
            date_time = now.strftime("%Y-%m-%d_%H-%M-%S")
            fn = 'Line_Travel_Data_'+date_time+'.txt'
            np.savetxt(fn,data)
            print('Data saved to',fn)
            
    def get_target_center(self,im, hsv_lower_bound,hsv_upper_bound):
        im_hsv = cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
        b1 = hsv_lower_bound[0]
        b2 = hsv_upper_bound[0]
        mask = cv2.inRange(im_hsv,b1,b2)
        for i in np.arange(1,len(hsv_lower_bound)):
            mask += cv2.inRange(im_hsv,hsv_lower_bound[i],hsv_upper_bound[i])
        size = np.sum(mask==255)
        if size>0:
            M = cv2.moments(mask)
            # calculate x,y coordinate of center
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]
            #cX = cX
            #cY = cY
            #print(cX,cY,size)
            size_re = size / im.shape[0] / im.shape[1]
            return (cX,cY),size_re,mask
        else:
            return (-1,-1),0.0,None
    
    def get_color_mask(self,im,color,cvtHSV=1):
        if cvtHSV==1:
            im = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        hsv_lower_bound,hsv_upper_bound = self.get_target_hsv_bound(color)
        b1 = hsv_lower_bound[0]
        b2 = hsv_upper_bound[0]
        mask = cv2.inRange(im,b1,b2)
        for i in np.arange(1,len(hsv_lower_bound)):
            mask_ = cv2.inRange(im,hsv_lower_bound[i],hsv_upper_bound[i])
            mask = cv2.bitwise_or(mask,mask_)
        return mask

    def get_target_hsv_bound(self,color):
        if self.location=='home':

            # Tej home
            # if color=='red':
            #     return [(150,120,0),(0,120,0)],[(180,255,255),(20,255,255)]
            # if color=='green':
            #     return [(40,70,50)],[(100,255,255)]
            # if color=='blue':
            #     return [(100,120,100)],[(130,255,255)]
            # if color=='yellow':
            #     return [(20,100,100)],[(40,255,255)]
            # if color=='black':
            #     return [(100,20,0)],[(120,255,100)]

            # home room mask
            # if color=='red':
            #     return [(150,100,0),(0,100,0)],[(180,255,255),(20,255,255)]
            # if color=='green':
            #     return [(40,50,50)],[(90,255,255)]
            # if color=='blue':
            #     return [(90,120,100)],[(120,255,255)]
            # if color=='yellow':
            #     return [(20,80,100)],[(40,255,255)]
            # if color=='black':
            #     return [(0,0,0)],[(255,255,120)]

            # Binghan room mask
            # if color=='red':
            #     return [(150,120,50),(0,120,50)],[(180,255,255),(20,255,255)]
            # if color=='green':
            #     return [(40,80,40)],[(90,255,255)]
            # if color=='blue':
            #     return [(90,100,70)],[(120,255,255)]
            # if color=='yellow':
            #     return [(20,80,100)],[(40,255,255)]
            # if color=='black':
            #     return [(0,0,0)],[(255,255,120)]

            # home living mask
            if color=='red':
                return [(150,150,50),(0,150,50)],[(180,255,255),(20,255,255)]
            if color=='green':
                return [(40,50,10)],[(90,255,255)]
            if color=='blue':
                return [(110,140,50)],[(130,255,150)]
            if color=='yellow':
                return [(20,80,100)],[(40,255,255)]
            if color=='yellow':
                return [(20,80,100)],[(40,255,255)]
            if color=='black':
                return [(90,0,0)],[(150,255,120)]

        elif self.location=='umd':
            if color=='red':
                return [(150,100,120),(0,100,120)],[(180,255,255),(10,255,255)]
            if color=='green':
                return [(40,50,50)],[(90,255,255)]
            if color=='blue':
                return [(90,120,100)],[(120,255,255)]
            if color=='yellow':
                return [(20,80,100)],[(40,255,255)]
            if color=='black':
                return [(0,0,0)],[(255,255,120)]
    
    def find_largest_component(self,mask): # for turning to target
        h,w = mask.shape
        numLabels, labels, stats, centroid = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)
        if numLabels==1: # only background
            return (-1,-1)
        cX = centroid[:,0]
        cY = centroid[:,1]
        # find the largest component that is not the background        
        max_ind = np.argmax(stats[1:,cv2.CC_STAT_AREA])+1
        size_normalize = stats[max_ind,cv2.CC_STAT_AREA]/h/w
        # print('normalized size', size_normalize)
        if size_normalize > self.size_threshold:
            return (cX[max_ind],cY[max_ind])
        else:
            return (-1,-1)

    def find_largest_component_close_to_previous(self,mask,prev_coord): # for turning to target
        h,w = mask.shape
        numLabels, labels, stats, centroid = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)
        if numLabels==1: # only background
            return (-1,-1)
        cX = centroid[:,0]
        cY = centroid[:,1]
        # find the largest component that is not the background
        if prev_coord[0] == -1:  # -1 means no previous data
            ind = np.argmax(stats[1:,cv2.CC_STAT_AREA])+1
            
        else: # use find cX, cY close to the prev_coord
            isort = np.flip( np.argsort(stats[1:,cv2.CC_STAT_AREA])+1 )
            if isort.size>=2:
                (cx_prev, cy_prev) = prev_coord
                # compare cX and cY
                cx0,cy0 = centroid[isort[0],:] # biggest
                cx1,cy1 = centroid[isort[1],:] # second biggest
                dist0 = np.linalg.norm([cx0-cx_prev, cy0-cy_prev])
                dist1 = np.linalg.norm([cx1-cx_prev, cy1-cy_prev])
                # print(np.round(cx_prev,2), np.round(cy_prev,2), np.round(cx0,2), np.round(cy0), np.round(cx1,2),np.round(cy1,2))
                if dist0 <= dist1:
                    ind = isort[0]
                else:
                    ind = isort[1]
            else:
                ind = isort[0]

        size_normalize = stats[ind,cv2.CC_STAT_AREA]/h/w
        # print('normalized size', size_normalize)
        if size_normalize > self.size_threshold:
            return (cX[ind],cY[ind])
        else:
            return (-1,-1)
    
    def find_component_closest_to_center(self,mask): # for turning to target
        h,w = mask.shape
        numLabels, labels, stats, centroid = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)
        cX = centroid[:,0]
        cY = centroid[:,1]
        min_ind = np.argmin(np.abs(cX-self.desired_center_x))
        # find the largest component that is not the background        
        size_normalize = stats[min_ind,cv2.CC_STAT_AREA]/h/w
        if size_normalize > self.size_threshold:
            return (cX[min_ind],cY[min_ind])
        else:
            return (-1,-1)
    
    def apply_homography(self,H,pt,mode='pixel'):
        n = pt.shape[0]
        pt_h = np.zeros((3,n))
        pt_h[:2,:] = pt.T
        pt_h[2,:] = 1.0
        pt_m = H.dot(pt_h)
        if mode=='pixel':
            pt_m = np.round(pt_m[:2,:]/pt_m[2,:]).astype(int)
        else:
            pt_m = pt_m[:2,:]/pt_m[2,:]
        return pt_m.T

    def find_block_position_cam_frame(self,im,niter=1,target_color='all'):
        H = np.array([[-0.0062,0.1869,-117.4956],[0.1428,-0.0089,-59.5014],[0.0001,-0.0104,1.]])
        fy = 664.87 # focal length in pixel in the y direction
        max_dist = 300 # ignore anything detected beyond this distance

        # resize to speed up things
        resize_factor = 2
        im = self.resize_im(im, resize_factor)
        
        im_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        h,w = im.shape[:2]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        if target_color=='all':
            colors = self.target_colors
        else:
            colors = [target_color]
        block_pos_world = dict()
        block_pos_pixel = dict()
        block_dist = dict()
        masks = dict()
        for color in colors:
            pt_world = []
            block_dist_color = []
            mask = self.get_color_mask(im_hsv,color,cvtHSV=0)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            numLabels, labels, stats, centroid = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)

            mask = cv2.GaussianBlur(mask,(5,5),cv2.BORDER_DEFAULT)
            masks[color] = mask
            
            # print('center',centroid[1:,:])
            for i in np.arange(1,numLabels):
                size_normalize = stats[i,cv2.CC_STAT_AREA]/h/w

                # print('component',i,'size',size_normalize)
                if size_normalize < self.size_threshold:
                    # print('   too small, skipped')                    
                    continue

                # estimate distance from block height
                block_height = stats[i, cv2.CC_STAT_HEIGHT]

                if color=='black':
                    bb_x = stats[i, cv2.CC_STAT_LEFT]
                    bb_y = stats[i, cv2.CC_STAT_TOP]
                    bb_w = stats[i, cv2.CC_STAT_WIDTH]
                    bb_h = stats[i, cv2.CC_STAT_HEIGHT]
                    bb_tol = 3
                    # if the bounding box is touching the edge of the image, ignore
                    if size_normalize>0.05 or (bb_x<=bb_tol or bb_x+bb_w>=w-bb_tol or bb_y<=bb_tol or bb_y+bb_h>=h-bb_tol):
                        # print('   too big or background, skipped')  
                        continue

                distance_est = fy / block_height * 5.715
                
                mask_comp = np.array((labels==i)*255,dtype=np.uint8)
                mask_comp = cv2.GaussianBlur(mask_comp,(3,3),cv2.BORDER_DEFAULT)
                corners_all = None
                for iter in range(niter):
                    corners = cv2.goodFeaturesToTrack(np.float32(mask_comp), 20, 0.04, 1)
                    if corners is None:
                        continue
                    corners = np.squeeze(np.int0(corners))
                    if corners.size==2:
                        corners = corners.reshape((1,2)) # ensure 2d 
                    if corners_all is None:
                        corners_all = corners
                    else:
                        corners_all = np.append(corners_all,corners,axis=0)
                
                if corners_all is None:
                    continue
                # find the lowest n points from corners_all
                isort = np.flip(np.argsort(corners_all[:,1]))[:3*niter]
                lowest_pt = corners_all[isort,:].reshape((isort.size,2)) # enforce 2D
                # rescale
                lowest_pt = lowest_pt*resize_factor
                lowest_pt_world = self.apply_homography(self.H,lowest_pt,mode='world')
                # take median
                m_x,m_y = np.median(lowest_pt_world,axis=0)
                if m_x<0: # skip negative points
                    continue
                # ignore point if it is too close (when picked up)
                too_close_dist = 8
                if m_x**2 + m_y**2 < too_close_dist**2:
                    continue

                # ignore if too far away (the estimate is not accurate)
                if m_x**2 + m_y**2 > max_dist**2:
                    continue

                pt_world.append(np.array([m_x,m_y]) )
                block_dist_color.append(distance_est)

            block_pos_world[color] = pt_world
            block_pos_pixel[color] = self.apply_homography(self.H_inv,np.array(pt_world),mode='pixel')
            block_dist[color] = block_dist_color

        return block_pos_world, block_pos_pixel, block_dist, masks
    
    def split_target_other_block(self, block_pos_world,target_color, mode='target_block'):
        target_block_pos = []
        other_block_pos = []
        for color, pts in block_pos_world.items():
            for i, pt in enumerate(pts):
                if mode == 'target_block' and color == target_color:
                    target_block_pos.append(pt)
                else:
                    other_block_pos.append(pt)
        target_block_pos = np.array(target_block_pos)
        other_block_pos = np.array(other_block_pos)

        if target_block_pos.shape[0]>0:
            # find minimum
            dist_sq = target_block_pos[:,0]**2 + target_block_pos[:,1]**2
            min_ind = np.argmin(dist_sq)
            target_block_pos_tmp = target_block_pos[[min_ind],:]
            target_block_pos = np.delete(target_block_pos, min_ind,axis=0)
            if other_block_pos.size==0:
                other_block_pos = target_block_pos
            else:
                other_block_pos = np.append(other_block_pos,target_block_pos,axis=0)
            target_block_pos = target_block_pos_tmp
        
        return target_block_pos, other_block_pos

    def plot_block_position(self,im, block_pos_camfr, block_pos_pixel):
        for (color, pixel),(_,pt_camfr) in zip(block_pos_pixel.items(),block_pos_camfr.items()):
            if len(pt_camfr)==0: # empty list
                continue

            if color=='red':
                c = [0,0,255]
            elif color=='green':
                c = [0,255,0]
            elif color=='blue':
                c = [255,0,0]
            elif color=='yellow':
                c = [0,255,255]
            elif color=='black':
                c = [0,0,0]
            
            for i in range(len(pt_camfr)):
                x,y = pt_camfr[i] # pt_camfr is a list
                x += self.cam_base_xoffset # convert to base frame 
                p_x,p_y = pixel[i,:] # pixel is a np array
                im = cv2.circle(im,(int(p_x),int(p_y)),5,c,-1)
                fontScale = 0.5
                thickness = 1
                im = cv2.putText(im,'({:.1f},{:.1f})'.format(x,y),(int(p_x),int(p_y)+20),
                                 cv2.FONT_HERSHEY_SIMPLEX,fontScale,c,thickness,cv2.LINE_AA)
        return im

    def constrain_turn_P(self,P,ind):
        sn = np.sign(P)
        absP = abs(P)
        absP = max(self.min_turn_pwm[ind],absP)
        absP = min(self.max_turn_pwm[ind],absP)
        return absP*sn
    
    def save_image_with_action(self,im, target_color, block_id, action, ind=0):
        print('saving planning image...')
        im = self.add_caption(im,caption='{} block {}. robot location: ({:.1f},{:.1f})'.format(target_color,action,self.curr_x, self.curr_y))
        action_ = action.replace(' ','_')
        # save image to disk
        subject = '{}_block_{:d}_{}'.format(target_color, int(np.floor(block_id/3))+1, action_)
        if ind>0:
            subject = subject+'_{:d}'.format(ind)

        fn = subject+'.jpg'
        cv2.imwrite(fn, im)
        time.sleep(0.1)
        return subject, fn

    def send_email(self,subject, im_fn):
        if self.do_send_email==0:
            return
        try:
            print('sending email...')
            # self.msgr.send(subject=subject,contents=None,attachments=im_fn)
            self.msgr.send_now(subject=subject,attachments=im_fn)
        except:
            pass

    def retrieve_target(self, picam2mt, monitor, target_color, gripper, block_id=0,debug=0):
        success_1 = self.face_target(picam2mt, monitor, target_color, mode='fine', verbose=0)
        if success_1 == 1:
            gripper.open_gripper()
            self.reach_for_target(picam2mt, monitor, target_color,debug=debug)
            gripper.close_gripper()
            time.sleep(self.pause_time*2)
            # send email
            im = picam2mt.frame
            subject, im_fn = self.save_image_with_action(im, target_color, block_id, 'picked up')
            self.send_email(subject, im_fn)
            return 1
        else:
            return 0

    def get_target_in_view_v2(self,target_color, picam2mt, monitor, debug=0):
        # use find_block_position_cam_frame to get block coordinates (target color only)
        # turn to some predefined positions to look for target block
        print('Getting target in view...')
        arena_center = np.array([[6,4]])*30.48
        unit_offset = 80
        offset = np.array([[1, 0],
                           [0,-1],
                           [-1,0],
                           [0, 1]])*unit_offset
        look_location_list = arena_center + offset
        
        iloc = 0
        niter = 0
        while True:
            # try turning to different locations
            print('Looking towards location',iloc,look_location_list[iloc])
            _ = self.turn_towards_location(monitor,look_location_list[iloc])
            # time.sleep(0.5)
            im = picam2mt.frame # get current image
            block_pos_camfr, block_pos_pixel, _, _ = self.find_block_position_cam_frame(im,target_color=target_color)
            target_block_pos_camfr, other_pos = self.split_target_other_block(block_pos_camfr,target_color,mode='target_block')
            if debug==1:
                im = self.plot_block_position(im, block_pos_camfr, block_pos_pixel)
                cv2.imshow('Camera', im)
                cv2.waitKey(0)

            print('Target block pos cam frame:', target_block_pos_camfr.flatten())
            
            # split_target_other_block will return the target block closest to the robot, if it is found
            if target_block_pos_camfr.size>0:
                # turn towards coordinate
                target_block_pos_basefr = self.camfr_to_basefr(target_block_pos_camfr)
                target_pos_worldfr = self.convert_to_world_frame(monitor, target_block_pos_basefr)
                _ = self.turn_towards_location(monitor, target_pos_worldfr)
                print('Target in view with high probability')
                return 1
            else: # did not find target
                # continue with other locations, or reposition if needed
                iloc += 1
                print('Target not in view, checking location',iloc)
                if iloc >= look_location_list.shape[0]:
                    # reached the end of the list, try reposition by moving towards the center of the arena
                    self.reposition(monitor,reposition_dist=50)
                    iloc = 0 # reset
                    niter += 1
                    if niter >= 5:
                        return 0

    def face_target(self, picam2mt,monitor,target_color,mode='fine',verbose=0,debug=0): 
        # this function is for when we are quite close to the actual target

        # get hsv bound based on target color
        hsv_lower_bound, hsv_upper_bound = self.get_target_hsv_bound(target_color)
        # initialize parameters
        err_turn_sum = 0.
        err_turn_diff = 0.
        err_turn_last = 0.

        init = 1
        last_timestamp = picam2mt.frame_timestamp
        in_range_cnt_turn = 0
        in_range_cnt = 0
        speedL = 0
        speedR = 0

        # a list of spin degrees to try when target not in view
        spin_degree_list = [0,-30,30,60,-60]
        spin_ind = 0
        not_found_counter = 0
        cX = -1
        cY = -1
        while True:
            curr_timestamp = picam2mt.frame_timestamp
            if curr_timestamp < last_timestamp+100:
                continue
                                
            # new frame
            last_timestamp = curr_timestamp
            im = picam2mt.frame
            im = self.resize_im(im, factor=4)
            mask = self.get_color_mask(im,target_color)
            (cX,cY) = self.find_largest_component_close_to_previous(mask, prev_coord=(cX,cY))
            # print(cX,cY)
            if cX == -1:
                not_found_counter += 1
            else:
                not_found_counter = 0
            
            if not_found_counter > 10:
                print('target not found! trying reversing and spinning...')
                if spin_ind < len(spin_degree_list):
                    # try spin and see if we can find target
                    if spin_ind==0:
                        assert(spin_degree_list[0]==0)
                        # the first action to try is reversing
                        self.forward_controller(max_dist=30,backwards=True,monitor=monitor)
                    else:
                        turn_degree = spin_degree_list[spin_ind] - spin_degree_list[spin_ind-1]
                    spin_ind += 1
                    self.turn_by_degree(monitor, turn_degree)
                else:
                    print('All actions tried, target still not found, returning to previous step')
                    success = 0
                    break

            if debug == 1:
                show_mask = 1
                if show_mask == 1:
                    im = mask
                im = cv2.line(im,(self.desired_center_x,0),(self.desired_center_x,im.shape[0]),(255,255,255),1)
                cv2.imshow('feed',im)
                cv2.waitKey(1)
            
            err_turn = self.desired_center_x - cX
            err_turn_last,err_turn_diff,err_turn_sum,init = self.update_PID_terms(err_turn,err_turn_last,err_turn_sum,init)
            
            P1 = self.constrain_turn_P(self.Kp_turn_pix*err_turn,0)
            D1 = self.Kd_turn_pix*err_turn_diff
            
            if mode=='fine':
                tol_pixel = self.tol_pixel
            else:
                tol_pixel = self.tol_pixel_coarse

            if abs(err_turn)<=tol_pixel:
                in_range_cnt_turn += 1
                speedL = 0
                speedR = 0
                if in_range_cnt_turn>=5:
                    success = 1
                    break
            else:
                in_range_cnt_turn = 0
                speedR = P1 + D1
                speedL = -speedR 

            self.set_speed_left(speedL)
            self.set_speed_right(speedR)

            if verbose==1:
                try:
                    print(np.round(err_turn,2),
                        in_range_cnt_turn,
                        np.round(speedL,2),np.round(speedR,2))
                except:
                    continue

        self.set_speed_left(0.)
        self.set_speed_right(0.)
        time.sleep(self.pause_time)
        return success
    
    def update_PID_terms(self,err,err_last,err_sum,init=1):
        err_sum = err_sum + err
        if init == 1:
            err_diff = 0
            init = 0
        else:
            err_diff = err-err_last
        err_last = err
        return err_last, err_diff, err_sum, init

    def resize_im(self, im, factor=4):
        im = cv2.resize(im,(int(im.shape[1]/factor),int(im.shape[0]/factor)))
        return im

    def reach_for_target(self, picam2mt, monitor, target_color,debug=0):
        # get hsv bound based on target color
        hsv_lower_bound, hsv_upper_bound = self.get_target_hsv_bound(target_color)

        # initialize parameters
        err_turn_sum = 0.
        err_turn_diff = 0.
        err_turn_last = 0.

        err_grab_sum = 0.
        err_grab_diff = 0.
        err_grab_last = 0.

        # init odom
        start_count_left = monitor.left_count.value
        start_count_right = monitor.right_count.value
        start_yaw = monitor.imu_reading.value
        # rot_mat = self.calculate_rot_matrix_from_rad(start_yaw)
        
        init = 1
        last_timestamp = picam2mt.frame_timestamp
        in_range_cnt_grab = 0
        speedL = 0
        speedR = 0
        cX = -1
        cY = -1
        while True:
            curr_timestamp = picam2mt.frame_timestamp
            if curr_timestamp <= last_timestamp:
                continue
                                
            # new frame
            last_timestamp = curr_timestamp
            im = picam2mt.frame
            im = self.resize_im(im,factor=4)
            mask = self.get_color_mask(im,target_color)
            (cX,cY) = self.find_largest_component_close_to_previous(mask,prev_coord=(cX,cY))

            if debug==1:
                show_mask = 1
                if show_mask == 1:
                    im = mask
                im = cv2.line(im,(self.desired_center_x,0),(self.desired_center_x,im.shape[0]),(255,255,255),1)
                cv2.imshow('feed',im)
                cv2.waitKey(1)

            # curr_yaw = monitor.imu_reading.value
            # LC = monitor.left_count.value
            # RC = monitor.right_count.value
            # if init == 0:
            #     # update odom
            #     self.update_curr_pos(LC_prev,LC,RC_prev,RC,last_yaw)
            # last_yaw = curr_yaw
            # LC_prev = LC
            # RC_prev = RC
            
            # calculate error
            err_turn = self.desired_center_x - cX
            err_grab = self.desired_center_y - cY

            # updates
            err_turn_last,err_turn_diff,err_turn_sum,_ = self.update_PID_terms(err_turn,err_turn_last,err_turn_sum,init)
            err_grab_last,err_grab_diff,err_grab_sum,init = self.update_PID_terms(err_grab,err_grab_last,err_grab_sum,init)

            P1 = self.constrain_turn_P(self.Kp_turn_pix*err_turn,0)
            D1 = self.Kd_turn_pix*err_turn_diff

            P2 = self.Kp_grab*err_grab
            D2 = self.Kd_grab*err_grab_diff

            if abs(err_grab)>self.tol_pixel:
                in_range_cnt_grab=0

                speedR = (P1+D1)*0.4+P2+D2
                speedL = -(P1+D1)*0.4+P2+D2
                
                speedR = max(8,speedR)
                speedR = min(self.speed_sum_max/2,speedR)
                speedL = max(8,speedL)
                speedL = min(self.speed_sum_max/2,speedL)
                
            else:
                in_range_cnt_grab += 1
                if in_range_cnt_grab>=5:
                    break
            
            # set speed
            self.set_speed_left(speedL)
            self.set_speed_right(speedR)

        # retrieved target, stop 
        self.set_speed_left(0.)
        self.set_speed_right(0.)
        time.sleep(self.pause_time)

        # update odom one more time
        end_count_left = monitor.left_count.value
        end_count_right = monitor.right_count.value
        # print(start_count_left,end_count_left,start_count_right,end_count_right)
        self.update_curr_pos(start_count_left,end_count_left,
                            start_count_right,end_count_right,start_yaw)
        self.report_current_pose(monitor)
   
    def turn_towards_location(self, monitor, loc):
        if type(loc)==np.ndarray:
            loc = loc.flatten()
        else:
            loc = np.array(loc).flatten()
        dx = loc[0] - self.curr_x
        dy = loc[1] - self.curr_y
        new_heading = np.array([dx,dy])

        curr_yaw = monitor.imu_reading.value
        rot_mat = self.calculate_rot_matrix_from_rad(curr_yaw)

        turn_degree = self.calculate_deg_to_turn(rot_mat,new_heading)
        print('Degree to turn:',np.round(turn_degree))
        self.turn_by_degree(monitor,turn_degree)
        return turn_degree

    def ensure_continuity(self, curr_yaw, last_yaw):
        if last_yaw is None:
            return curr_yaw
        if np.abs(curr_yaw-last_yaw) < np.pi: # small change
            return curr_yaw
        test_yaw = np.array([-np.pi*2,0,np.pi*2])+curr_yaw
        imin = np.argmin(np.power(test_yaw-last_yaw,2))
        return test_yaw[imin]

    def turn_by_degree(self,monitor,turn_degree,verbose=0):
        time.sleep(0.5)
        if turn_degree is None:
            return
        turn_rad = turn_degree/180*np.pi
        start_yaw = monitor.imu_reading.value
        rot_mat = self.calculate_rot_matrix_from_rad(start_yaw)
        
        # initialize parameters
        err_rad_sum = 0.
        err_rad_diff = 0.
        err_rad_last = 0.
        init = 1
        
        last_time = time.time_ns()
        last_yaw_ref = None
        while True:
            curr_time = time.time_ns()
            if curr_time-last_time<=0.05*1e9:
                time.sleep(0.001)
                continue
            else:
                last_time = curr_time
                    
            curr_yaw = monitor.imu_reading.value
            curr_yaw_ref0 = self.convert_rad_in_ref(rot_mat,curr_yaw)
            # check continuity of yaw, problomatic only when the degree is close to pi or -pi
            curr_yaw_ref = self.ensure_continuity(curr_yaw_ref0, last_yaw_ref)
            # print(curr_yaw_ref0,curr_yaw_ref)
            err_rad = turn_rad - curr_yaw_ref
            if abs(err_rad) <= self.tol_rad:
                break
                
            err_rad_sum = err_rad_sum + err_rad
            if init == 0:
                err_rad_diff = err_rad - err_rad_last
            else:
                err_rad_diff = 0
                init = 0
            last_yaw_ref = curr_yaw_ref
            err_rad_last = err_rad
            
            # again use PID control
            P = self.constrain_turn_P(self.Kp_turn_rad*err_rad,1)
            D = self.Kd_turn_rad*err_rad_diff
            speedR = P + D
            speedL = -speedR
            
            self.set_speed_left(speedL)
            self.set_speed_right(speedR)
            if verbose==1:
                print(np.round(curr_yaw,2),np.round(err_rad/np.pi*180,2),
                    np.round(speedL,1),np.round(speedR,1))
        self.set_speed_left(0.)
        self.set_speed_right(0.)
        time.sleep(self.pause_time)

        self.report_current_pose(monitor)

    def turn_by_abs_yaw(self, monitor, target_yaw):
        c = np.cos(target_yaw)
        s = np.sin(target_yaw)
        new_heading = np.array([c,s])

        curr_yaw = monitor.imu_reading.value
        rot_mat = self.calculate_rot_matrix_from_rad(curr_yaw)

        turn_degree = self.calculate_deg_to_turn(rot_mat,new_heading)
        # print(turn_degree)
        self.turn_by_degree(monitor,turn_degree)

    def calculate_rotation_matrix(self,x,y):
        theta = np.arctan2(y,x)
        theta_orth = theta + np.pi/2
        rot_mat = np.array([[np.cos(theta),np.cos(theta_orth)],[np.sin(theta),np.sin(theta_orth)]])
        return rot_mat

    def calculate_deg_to_turn(self,rot_mat,new_heading):
        heading_hat = rot_mat.T.dot(new_heading.reshape((2,1))).flatten()
        theta = np.arctan2(heading_hat[1],heading_hat[0])
        return theta/np.pi*180

    def follow_path(self,monitor,path,mode='target_block'):
        if path is None:
            return
        for i in range(path.shape[0]-1):
            # first figure out how much to turn
            if i==0:
                curr_heading = np.array([1.0,0])
            else:
                curr_heading = new_heading
            new_heading = path[i+1,:]-path[i,:]
            rot_mat = self.calculate_rotation_matrix(curr_heading[0],curr_heading[1])
            turn_degree = self.calculate_deg_to_turn(rot_mat,new_heading)
            # perform turning
            self.turn_by_degree(monitor,turn_degree)
            # then go straight
            skip_fwd = 0
            if mode == 'target_block' and i+1==path.shape[0]-1:
                skip_fwd = 1
            if skip_fwd == 1:
                continue
            self.forward_controller(max_dist=np.linalg.norm(new_heading),
                                backwards=False,monitor=monitor,log_data=0)
            
    def plot_path_on_image(self, im, path, mode='target_block'):
        # remember path is in the base frame not cam frame
        # to apply homography, need cam frame
        offset_array = np.array([[-self.cam_base_xoffset,0]])
        for i in range(path.shape[0]-1):
            pt1_ = path[[i],:]+offset_array
            pt1_[0,0] = np.maximum(0., pt1_[0,0])
            pt2_ = path[[i+1],:]+offset_array
            pt2_[0,0] = np.maximum(0., pt2_[0,0])
            pt1 = self.apply_homography(self.H_inv,pt1_,mode='pixel').flatten()
            pt2 = self.apply_homography(self.H_inv,pt2_,mode='pixel').flatten()
            # if mode=='target_block':
            #     if i<path.shape[0]-2:
            #         color = [255,255,255]
            #     else:
            #         color = [0,0,255]
            # else:
            #     color = [255,255,255]
            im = cv2.line(im,(pt1[0],pt1[1]),(pt2[0],pt2[1]),[255,255,255],3)
            im = cv2.circle(im,(pt1[0],pt1[1]),10,[255,255,255],-1)
        return im
    
    def get_target_in_view(self,target_color,picam2mt,monitor):

        # turn to some predefined positions to look for target block
        center = np.array([[6,4]])*30.48
        unit_offset = 80
        offset = np.array([[1, 0],
                           [0,-1],
                           [-1,0],
                           [0, 1]])*unit_offset
        look_location_list = center + offset

        for loc in look_location_list:
            _ = self.turn_towards_location(monitor,loc)

            # check if target is in view
            im = picam2mt.frame
            im = self.resize_im(im, factor=4)
            mask = self.get_color_mask(im,target_color)
            (cX,cY) = self.find_largest_component(mask)
            print('coord',cX,cY)
            if cX != -1:
                return 1
            
        return 0

    def convert_to_base_frame(self, monitor, world_coord_input, mode='base'):
        world_coord = np.copy(world_coord_input)
        if world_coord.ndim == 1:
            world_coord = np.reshape(world_coord,(2,1))
        else:
            world_coord = world_coord.T 
            assert(world_coord.shape[0]==2)
        curr_yaw = monitor.imu_reading.value
        rot_mat = self.calculate_rot_matrix_from_rad(curr_yaw)
        t = np.array([self.curr_x, self.curr_y]).reshape((2,1))
        base_coord = np.linalg.multi_dot([ rot_mat.T, (world_coord-t) ])
        # offset if in cam mode
        if mode=='cam':
            base_coord[0,:] -= self.cam_base_xoffset

        return base_coord.T

    def convert_to_world_frame(self, monitor, base_coord_input):
        base_coord = np.copy(base_coord_input)
        if base_coord.ndim == 1:
            base_coord = np.reshape(base_coord,(2,1))
        else: # 2d array, transpose, 2 by N matrix after transpose
            base_coord = base_coord.T 
            assert(base_coord.shape[0]==2)
        curr_yaw = monitor.imu_reading.value
        rot_mat = self.calculate_rot_matrix_from_rad(curr_yaw)
        t = np.array([self.curr_x, self.curr_y]).reshape((2,1))
        world_coord = np.linalg.multi_dot([ rot_mat, base_coord]) + t
        return world_coord.T
    
    def find_drop_location(self, ind):
        return self.drop_loc[[ind],:]

    def reposition(self,monitor,reposition_dist=30,turn_back=0):
        print('repositioning...')
        degree_turned = self.turn_towards_location(monitor, [4*30.48,4*30.48])
        self.forward_controller(max_dist=reposition_dist,backwards=False,monitor=monitor)
        if turn_back==1:
            self.turn_by_degree(monitor, -degree_turned)

    def camfr_to_basefr(self, camfr_pt):
        basefr_pt = camfr_pt.copy()
        if basefr_pt.size>0: # 2d array
            basefr_pt[:,0] += self.cam_base_xoffset
        return basefr_pt
    
    def basefr_to_camfr(self, basefr_pt):
        camfr_pt = basefr_pt.copy()
        if camfr_pt.size>0: # 2d array
            camfr_pt[:,0] -= self.cam_base_xoffset
        return camfr_pt

    def add_caption(self, im, caption):
        im = cv2.putText(im,caption,(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.8,[255,255,255],2,cv2.LINE_AA)
        return im

    def drive_close_to_block(self, picam2mt, monitor, target_color, planner, block_id=0, debug=0):
        dist_threshold = 100+self.cam_base_xoffset
        break_dist_block = 50

        target_pos_worldfr = None
        path_to_block_followed = 0
        ipath = 0
        while True:

            # determine block locations
            im = picam2mt.frame # get current image
            block_pos_cam_frame, block_pos_pixel, _, _ = self.find_block_position_cam_frame(im)
            target_pos_camfr, other_pos_camfr = self.split_target_other_block(block_pos_cam_frame,target_color,mode='target_block')
            target_pos_basefr = self.camfr_to_basefr(target_pos_camfr)
            other_pos_basefr = self.camfr_to_basefr(other_pos_camfr)

            # remember target_pos_camfr is in terms of the current body frame

            print('Target block pos cam frame', np.round(target_pos_camfr,2) )
            print('Other block pos cam frame\n', np.round(other_pos_camfr,2) )

            if debug==1:
                im = self.plot_block_position(im, block_pos_cam_frame, block_pos_pixel)
                cv2.imshow('Camera', im)
                cv2.waitKey(0)

            if target_pos_camfr.size == 0:
                print('Failed to detect target...')
                return 0
            
            target_pos_worldfr = self.convert_to_world_frame(monitor, target_pos_basefr)
            
            print('Target block pos body frame', np.round(target_pos_basefr,2) )
            print('Target block pos world frame', np.round(target_pos_worldfr,2) )

            # ++++++++ IMPORTANT +++++++++++++++++++++++++++++++++++++++++
            # ++++++++ target_pos_basefr and other_pos are in body frame ++++++++
            
            # find distance to the target block
            dist_to_target = np.linalg.norm(target_pos_basefr)

            # break if the target is close enough
            if dist_to_target < break_dist_block:
                print('Target distance {:.2f} within range'.format(dist_to_target))
                break

            # find distance to the other blocks (considered as obstacles)
            min_other_dist = 1e9
            # other blocks might be empty
            if other_pos_basefr.size>0:
                # if we see other blocks 
                min_other_dist = np.min( np.linalg.norm(other_pos_basefr,axis=1) )
            
            # plan path to drive closer to target
            if dist_to_target > dist_threshold and min_other_dist > dist_threshold:
                # no blocks near the robot, drive straightf
                self.forward_controller(max_dist=80,backwards=False,monitor=monitor)
            else:
                if dist_to_target > dist_threshold and min_other_dist < dist_threshold:
                    # pick an open space in the map that's closest to the target, and plan
                    # the path there
                    print('planning to open space...')
                    path_mode = 'open_space'
                    
                elif dist_to_target < dist_threshold:
                    print('planning to block...')
                    path_mode = 'target_block'

                path_trimmed, path_untrimmed, is_straight_path = \
                    planner.gen_path(target_pos_basefr,other_pos_basefr,mode=path_mode,
                                    curr_pose=(self.curr_x,self.curr_y,monitor.imu_reading.value),
                                    debug=debug)
                
                print('trimmed path\n',path_trimmed)
                print('is straight path:',is_straight_path)

                if path_trimmed is not None:
                    ipath += 1
                    if self.save_planning_img == 1:
                        im = self.plot_path_on_image(im, path_trimmed,path_mode)
                        subject, im_fn = self.save_image_with_action(im, target_color, block_id, 'retrieval path planning',ind=ipath)
                        if self.send_planning_email==1:
                            self.send_email(subject, im_fn)
                    if debug==1:
                        cv2.imshow('Camera', im)
                        cv2.waitKey(0)
                    if is_straight_path==1 and path_mode=='target_block':
                        print('Straight path, go right to retrieving')
                        break
                    
                    self.follow_path(monitor,path_trimmed,path_mode)
                    if path_mode=='target_block':
                        path_to_block_followed = 1
                else:
                    # reposition slightly
                    self.reposition(monitor,turn_back=1)
            
            # face the target once again after carrying out path
            if target_pos_worldfr is not None:
                # if we have a pervious estimate of target block position, 
                # we can calculate how much we need to turn from that position
                self.report_current_pose(monitor)
                print('Turning to target block pos world frame', np.round(target_pos_worldfr,2) )
                _ = self.turn_towards_location(monitor, loc=target_pos_worldfr)

            else:
                # use camera to center the block in the view 
                self.face_target(picam2mt,monitor,target_color,mode='coarse')

            if path_to_block_followed == 1:
                print('Path to block followed, proceed to retrieve target')
                break

        return 1

    def remove_block_in_czone(self, block_pos_worldfr):
        n_block = block_pos_worldfr.shape[0]
        robot_pos_worldfr = np.array([[self.curr_x,self.curr_y]])
        dist_block_czone = np.linalg.norm(block_pos_worldfr-self.czone_center_worldfr, axis=1) # from blocks to czone
        dist_block_robot = np.linalg.norm(block_pos_worldfr-robot_pos_worldfr, axis=1) # from block to robot
        dist_robot_czone = np.linalg.norm(robot_pos_worldfr-self.czone_center_worldfr) # from robot to czone
        block_keep = []
        block_robot_dist_keep = []
        block_removed = []
        for i in range(n_block):
            if dist_block_czone[i] < 2*30.48 or dist_block_robot[i]>dist_robot_czone:
                # if distance from block to block center is within 2ft, or if the distance of the block 
                # to the robot is even further than the distance from robot to the center of the constructin
                # zone, then skip this block
                block_removed.append(block_pos_worldfr[i,:])
                continue
            else:
                block_keep.append(block_pos_worldfr[i,:])
                block_robot_dist_keep.append(dist_block_robot[i])
        return np.array(block_keep), np.array(block_robot_dist_keep).flatten(),np.array(block_removed)

    def drive_to_czone(self, picam2mt, monitor,target_color, planner, block_id=0, debug=0):
        dist_threshold = 100+self.cam_base_xoffset
        break_dist_czone = 5
        ipath = 0
        while True:
            # find czone drop location
            czone_pos_worldfr = self.find_drop_location(block_id)
            print('+++++ Drop location,',czone_pos_worldfr)
            # calculate distance to construction zone drop location
            robot_pos_worldfr = np.array([[self.curr_x,self.curr_y]])
            dist_to_czone_worldfr = np.linalg.norm(czone_pos_worldfr-robot_pos_worldfr)

            if dist_to_czone_worldfr < break_dist_czone:
                # assume at czone already
                print('Arrived at construction zone!')
                break

            # turn towards the construction zone
            _ = self.turn_towards_location(monitor, loc=czone_pos_worldfr)

            czone_pos_basefr = self.convert_to_base_frame(monitor, czone_pos_worldfr)

            print('czone world coord: ',czone_pos_worldfr)
            print('czone base coord: ', czone_pos_basefr)

            # determine block locations
            im = picam2mt.frame # get current image
            block_pos_camfr, block_pos_pixel, _, _ = self.find_block_position_cam_frame(im)
            if debug==1:
                im = self.plot_block_position(im, block_pos_camfr, block_pos_pixel)
                cv2.imshow('Camera', im)
                cv2.waitKey(0)

            # now that we are driving to construction zone, all blocks are lumped into other_pos
            _, other_pos_camfr = self.split_target_other_block(block_pos_camfr,target_color=None, mode='target_czone')

            # convert to world frame
            other_pos_worldfr = np.array([])
            if other_pos_camfr.size>0:
                other_pos_basefr = self.camfr_to_basefr(other_pos_camfr)
                other_pos_worldfr = self.convert_to_world_frame(monitor, other_pos_basefr)
                other_pos_worldfr,block_robot_dist,block_removed = self.remove_block_in_czone(other_pos_worldfr)

                print('blocks kept: \n',other_pos_worldfr)
                print('blocks removed: \n',block_removed)

            if other_pos_worldfr.size==0: 
                # no obstacle in the path, can drive straight
                self.forward_controller(max_dist=dist_to_czone_worldfr,backwards=False,monitor=monitor)
                break
            else: # requires planning
                
                print('other block(s) pos base frame\n', np.round(other_pos_basefr, 2))
                
                min_other_dist = np.min(block_robot_dist)
                if dist_to_czone_worldfr > dist_threshold and min_other_dist > dist_threshold:
                    # no blocks near the robot, drive straight
                    self.forward_controller(max_dist=80,backwards=False,monitor=monitor)
                else:
                    if dist_to_czone_worldfr > dist_threshold and min_other_dist < dist_threshold: 
                        # pick an open space in the map that's closest to the target, and plan
                        # the path there
                        print('planning to open space...')
                        path_mode = 'open_space'
                        
                    elif dist_to_czone_worldfr < dist_threshold:
                        print('planning to czone...')
                        path_mode = 'target_czone'

                    path_trimmed, path_untrimmed,_ = planner.gen_path(czone_pos_basefr,other_pos_basefr,mode=path_mode,debug=debug)
                    print('trimmed path',path_trimmed)
                    # print('original path',path_untrimmed)

                    if path_trimmed is not None:
                        ipath += 1
                        if self.save_planning_img==1:
                            im = self.plot_path_on_image(im, path_trimmed,path_mode)
                            subject, im_fn = self.save_image_with_action(im, target_color, block_id, 'delivery path planning',ind=ipath)
                            if self.send_planning_email==1:
                                self.send_email(subject, im_fn)
                        if debug==1:
                            cv2.imshow('Camera', im)
                            cv2.waitKey(0)
                        self.follow_path(monitor,path_trimmed,path_mode)
                    else:
                        # reposition slightly
                        print('repositioning...')
                        self.reposition(monitor,turn_back=1)
        return 1

    def release_block(self, monitor, gripper):
        # turn to a speicific direction when drop the block
        self.turn_by_abs_yaw(monitor, target_yaw=-np.pi) 
        gripper.open_gripper()
        time.sleep(self.pause_time)
        gripper.relax()
        time.sleep(self.pause_time)
        # drive backwards
        self.forward_controller(max_dist=15,backwards=True,monitor=monitor)
        gripper.close_gripper()
    
    def relocalization(self,monitor):
        # after release the block, the robot is facing negative x direction
        # do a distance measure
        dist_x,_ = self.dist_sensor.get_distance()
        
        # turn 90 degrees to the right, now facing positive y direction
        self.turn_by_abs_yaw(monitor, 90/180*np.pi)
        dist_y,_ = self.dist_sensor.get_distance()

        print('Measured distance: ({:.2f},{:.2f})'.format(dist_x,dist_y))

        # update if both measurement valid
        if dist_x != -1 and dist_y != -1:
            self.curr_x = self.arena_xrange[0] + dist_x
            self.curr_y = self.arena_yrange[1] - dist_y
            self.report_current_pose(monitor)
            if self.save_position_data==1:
                self.append_curr_pos()