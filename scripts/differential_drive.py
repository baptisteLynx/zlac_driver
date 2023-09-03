"""
BSD 3-Clause License
Copyright (c) 2022, Mohamed Abdelkader Zahana
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import logging
from math import *
import numpy as np
import rospy

class DiffDrive:
    """Differential drive kinematics for 2-wheel robot
    """

    def __init__(self, wheel_radius, track_width):
        self._wheel_radius = wheel_radius
        self._track_width = track_width 
        self._odom = {'x':0,'y':0,'yaw':0,'v':0,'w':0}

        self.path = "None"

        self._l_pos = 0 # Left encoder position
        self._r_pos = 0 # Right right
        self._l_vel = 0 # left wheel angular velocity in rad/s
        self._r_vel = 0

    def calcWheelVel(self,v,w):
        """Calculates the left and right wheel speeds in rad/s from vx and w

        Parameters
        --
        @param v linear/forward velocity (v_x) in robot frame, m/s
        @param w Robot's angular velocity in rad/s

        Returns
        --
        @return wl Left wheel velocity in rad/s
        @return wr Right wheel velocity in rad/s
        """
        wr = 1/self._wheel_radius *(v + w * self._track_width/2)
        wl = 1/self._wheel_radius *(v - w * self._track_width/2)
        return (wl,wr)
    
    def calcRobotOdom(self, dt):
        """calculates linear and angular states from the left and right wheel speeds

        Parameters
        --
        @param dt time stamp in seconds
        """

        vl = self._l_vel
        vr = self._r_vel

        try:
            if (vl > 0.0) and (vr < 0.0) and (abs(vl) == abs(vr)):
                ## rotatiing CW
                linear_vel = 0
                angular_vel = (vl-vr)/self._track_width
                self._odom['yaw'] = self._odom['yaw'] + angular_vel*dt

                self.path = "skid_right"

            elif (vr > 0.0) and (vl < 0.0) and (abs(vl) == abs(vr)):
                ## rotatiing CCW
                linear_vel = 0
                angular_vel = (vr-vl)/self._track_width
                self._odom['yaw'] = self._odom['yaw'] + angular_vel*dt

                self.path = "skid_left"

            elif abs(vl) > abs(vr):
                ## curving CW
                linear_vel = (vl + vr)/2.0
                angular_vel = (vl-vr)/self._track_width
                R_ICC = (self._track_width/2.0)*((vl+vr)/(vl-vr))

                self._odom['x'] = self._odom['x'] - R_ICC*np.sin(self._odom['yaw']) + R_ICC*np.sin(self._odom['yaw'] + angular_vel*dt)
                self._odom['y'] = self._odom['y'] + R_ICC*np.cos(self._odom['yaw']) - R_ICC*np.cos(self._odom['yaw'] + angular_vel*dt)
                self._odom['yaw'] = self._odom['yaw'] - angular_vel*dt

                self.path = "curve_right"

            elif abs(vl) < abs(vr):
                ## curving CCW
                linear_vel = (vl + vr)/2.0
                angular_vel = (vr-vl)/self._track_width
                R_ICC = (self._track_width/2.0)*((vr+vl)/(vr-vl))

                self._odom['x'] = self._odom['x'] - R_ICC*np.sin(self._odom['yaw']) + R_ICC*np.sin(self._odom['yaw'] + angular_vel*dt)
                self._odom['y'] = self._odom['y'] + R_ICC*np.cos(self._odom['yaw']) - R_ICC*np.cos(self._odom['yaw'] + angular_vel*dt)
                self._odom['yaw'] = self._odom['yaw'] + angular_vel*dt

                self.path = "curve_left"

            elif vl == vr:
                linear_vel = (vl + vr)/2.0
                angular_vel = 0.0
                self._odom['x'] = self._odom['x'] + linear_vel*np.cos(self._odom['yaw'])*dt
                self._odom['y'] = self._odom['y'] + linear_vel*np.sin(self._odom['yaw'])*dt
                self._odom['yaw'] = self._odom['yaw']
                self.path = "straight"

            else:
                linear_vel = 0.0
                angular_vel = 0.0
                R_ICC = 0.0
        except Exception as e:
                rospy.logerr_throttle(1, "[Odom Calculation] Error in Calculation: %s", e)

        # Update odometry
        self._odom['v'] = linear_vel
        self._odom['w'] = angular_vel

        return self._odom

    def resetOdom(self):
        """Reset Odom to origin
        """
        self._odom = {'x':0,'y':0,'yaw':0,'v':0,'w':0}