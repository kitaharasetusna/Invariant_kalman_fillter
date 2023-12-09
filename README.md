# Invariant_kalman_fillter
A python implementation of right invariant kalman fillter; The case includes legged robots with IMU and contact sensors.


This is my course project for COMPSCI690K in UMASS Amherst. 

In this repository, I reimplemented the IEKF from [The Invariant Extended Kalman filter as a stable observerlink to a website][1].


## InEKF
To run the InEFK; The data cames from gazebo simulator provided in [this link. ](https://github.com/DAIRLab/cassie-gazebo-sim.git); For the forward kinematics,
we used urdf files from cassie robot to measure the length for all links to get SE(3) matrices to the hip.
```
python ./example.py
```
This part will demonstrate a 3D predicted trajectory for the cassie bipedal robot.
For the covariance matrix, we use identity matrcies to build it. The predicted curve should no obvious difference.
<img width="388" alt="image" src="https://github.com/kitaharasetusna/Invariant_kalman_fillter/assets/116760304/3ec638c8-2f8a-44af-bc57-3a5af385f240">


[1]: https://arxiv.org/pdf/1410.1465.pdf "Example Website"
