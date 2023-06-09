from picamera2 import Picamera2
from libcamera import Transform
from threading import Thread
from datetime import datetime
import numpy as np
import cv2

class Picam2MultiThread:
    def __init__(self,main_size=(640,480),lores_size=(160,120),
                framerate=25.0,
                verbose=False,
                rec_vid=0,
                vid_downsample_factor=3,
                vid_keepframe_itv=1,
                vid_framerate=25.0):
        frame_dur = int(1000000.0/framerate)
        self.cam = Picamera2()
        self.video_config = self.cam.create_video_configuration(main={"size":main_size,"format":"RGB888"},\
                                                                lores={"size":lores_size,"format":"YUV420"},\
                                                                transform=Transform(hflip=True,vflip=True))
        self.cam.configure(self.video_config)
        self.cam.set_controls({"FrameDurationLimits":(frame_dur,frame_dur)})
        self.frame = np.zeros((main_size[0],main_size[1],3),np.uint8)
        self.frame_lores = []
        self.metadata = []
        self.frame_timestamp = 0
        self.first_frame_timestamp = 0
        self.stopped = False
        self.total_frame = 0
        self.fps = 0.0
        self.verbose=verbose
        self.rec_vid = rec_vid
        self.framerate = framerate
        self.main_size = main_size
        self.vid_downsample_factor = vid_downsample_factor
        self.vid_keepframe_itv = vid_keepframe_itv
        self.vid_framerate = vid_framerate
    
    def start(self):
        print("camera streaming starting!")
        Thread(target=self.stream,args=()).start()
        return self
    
    def stream(self):
        if self.rec_vid == 1:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            now = datetime.now() # current date and time
            date_time = now.strftime("%Y-%m-%d_%H-%M-%S")
            vid_fn = date_time+'.avi'
            vidwriter = cv2.VideoWriter(vid_fn,fourcc,self.vid_framerate,(self.main_size[0],self.main_size[1]))
        self.cam.start()
        while not self.stopped:
            (frame,frame_lores),metadata = self.cam.capture_arrays(["main","lores"])
            last_timestamp = self.frame_timestamp
            self.frame = frame
            self.frame_lores = frame_lores
            self.frame_timestamp = metadata["SensorTimestamp"]
            if self.total_frame == 0:
                self.first_frame_timestamp = self.frame_timestamp
            else:
                self.fps = float(1e9/(self.frame_timestamp-last_timestamp))
            self.total_frame += 1
            self.metadata = metadata
            # print(metadata)
            if self.verbose:
                print(self.total_frame,self.frame_timestamp,self.fps)
            if self.rec_vid==1 and (self.total_frame-1)%self.vid_keepframe_itv==0:
                # frame_ds = self.resize_im(frame, self.vid_downsample_factor)
                vidwriter.write(frame)

        if self.rec_vid==1:
            vidwriter.release()
            print('video saved to',vid_fn)  
        # self.stopped becomes true
        self.cam.stop()
        print("camera streaming stopped")
    
    def resize_im(self, im, factor=4):
        im = cv2.resize(im,(int(im.shape[1]/factor),int(im.shape[0]/factor)))
        return im

    def stop(self):
        self.stopped = True
        
            
    


