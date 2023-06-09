# Autonomous target retrieval and delivery
This is the repository for all the code I developed for the course ENPM809T Autonomous Robot.

In this course I built and programmed an autonomous robot from scratch and the final task (grand challenge) is to autonomously search for and deliver red, green and blue blocks (in this sequence) to a construction zone. 

The motion controls are based IMU, encoder and camera information. The perception relies mainly on the camera images. To determine the block location in relation to the robot, I used corner detection and homography. With the block coordinates, path planning is performed to avoid non-target blocks using A* search algorithm. 

The video attached below details my journey through the course. Be sure to check it out!
[![ENPM809T](https://www.dropbox.com/s/boc8dndfhbv71my/ENPM809T.jpg?dl=0)](https://www.youtube.com/watch?v=_IznfzKgS2I "ENPM809T")
 
