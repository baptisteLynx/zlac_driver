#!/usr/bin/env python3

"""
ROS driver for the ZLAC8015D motor controller
"""

from time import time
import rospy
import tf
from zlac_driver.ZLAC8015D import Controller
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from differential_drive import DiffDrive
# from pid import PID
# from zlac8030l_ros.msg import State

class Driver:
    def __init__(self):
        self._port = rospy.get_param("~port", "/dev/ttyUSB0")
        self._accel_time = rospy.get_param("~accel_time", 1000)
        self._decel_time = rospy.get_param("~decel_time", 1000)
        self._slave_id = rospy.get_param("~slave_id", 1)

        self._wheel_ids = {"l":1, "r":2}
        self._flip_direction = {"l": 1, "r": -1}

        # Velocity vs. Troque modes
        self._speed_control = rospy.get_param("~speed_mode", True)

        # Stores current wheel speeds [rpm]
        self._current_whl_rpm = {"l": 0.0, "r": 0.0}
        # Target RPM
        self._target_whl_rpm = {"l": 0, "r": 0}
        # Target torque; when slef._control_mode="torque"
        self._target_current = {"l": 0.0, "r": 0.0}
        
        self._wheel_radius = rospy.get_param("~wheel_radius", 0.1016)

        self._track_width = rospy.get_param("~track_width", 0.45)

        self._max_vx = rospy.get_param("~max_vx", 1.5)
        self._max_w = rospy.get_param("~max_w", 1.0)

        # Max linear accelration [m/s^2] >0
        # self._max_lin_accel = rospy.get_param("~max_lin_accel", 10)
        # Max angular accelration [rad/s^2] >0
        # self._max_ang_accel = rospy.get_param("~max_ang_accel", 15)

        self._odom_frame = rospy.get_param("~odom_frame", "odom_link")
        self._robot_frame = rospy.get_param("~robot_frame", "base_link")

        self._loop_rate = rospy.get_param("~loop_rate", 100.0)

        self._cmd_timeout = rospy.get_param("~cmd_timeout", 0.1)

        self._diff_drive = DiffDrive(self._wheel_radius, self._track_width)

        # last time stamp. Used in odometry calculations
        self._last_odom_dt = time()

        # Last time a velcoity command was received
        self._last_cmd_t = time()

        # If True, odom TF will be published
        self._pub_tf = rospy.get_param("~pub_tf", False)

        self.motor_reset_alarm_conter = 0


        try: 
            self.motors = Controller(port=self._port)
            if (self._speed_control):
                mode=3
            else:
                mode=2
            self.motors_initilization(mode)  
            
        except Exception as e:
            rospy.logerr("Could not create motor object. Error: %s", e)
            exit(0)

        rospy.logwarn("\n ** cmd_vel must be published at rate more than %s Hz ** \n", 1/self._cmd_timeout)

        # ------------------- Subscribers ----------------#
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

        # ------------------- Publishers ----------------#
        self._odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        # self._vel_pub = rospy.Publisher("forward_vel", Float64, queue_size=10)

        rpm_left = rospy.Publisher("/zlac_driver/rpm/left", Float32, queue_size=10)
        rpm_right = rospy.Publisher("/zlac_driver/rpm/right", Float32, queue_size=10)
        # ------------------- Services ----------------#

        # TF broadcaster
        self._tf_br = tf.TransformBroadcaster()

        rospy.loginfo("** Driver initialization is done **\n")

    def motors_initilization(self, mode):
        """
        This method sets the accelaration and enables the motors.
        """
        # Disable motor
        self.motors.disable_motor()

        # Set acceleration and deceleration time
        self.motors.set_accel_time(self._accel_time,self._accel_time)
        self.motors.set_decel_time(self._decel_time,self._decel_time)

        # Set control mode
        self.motors.set_mode(mode)
        self.motors.enable_motor()

    def rpmToRps(self, rpm):
        return rpm / 9.5493

    def rpsToRpm(self, rad):
        return rad * 9.5493

    def check_driver_health(self):
        """
        This method checks the driver errors if any.
        """
        (L_fault, L_code),(R_fault, R_code) = self.motors.get_fault_code()

        if (L_fault or R_fault):
            rospy.loginfo(f"Motor is in fault state, Left Motor : {L_fault}, Right Motor : {R_fault}")
            rospy.loginfo(f"Motor fault code, Left Motor: {L_code}, Right Motor : {R_code}")

            if(L_code==self.motors.UNDER_VOLT or R_code==self.motors.UNDER_VOLT):
                rospy.loginfo(f"Motor fault reason - Under Voltage")
            elif(L_code==self.motors.OVER_CURR or R_code==self.motors.OVER_CURR):
                rospy.loginfo(f"Motor fault reason - Over Current")
            elif(L_code==self.motors.OVER_LOAD or R_code==self.motors.OVER_LOAD):
                rospy.loginfo(f"Motor fault reason - Over Load")
            elif(L_code==self.motors.MOTOR_BAD or R_code==self.motors.MOTOR_BAD):
                rospy.loginfo(f"Motor fault reason - Motor damage")
            elif(L_code==self.motors.HIGH_TEMP or R_code==self.motors.HIGH_TEMP):
                rospy.loginfo(f"Motor fault reason - High Temperature")

            rospy.loginfo(f"Resetting Motor fault.....")
            self.motors.clear_alarm()
            self.motor_reset_alarm_conter += 1 
            rospy.loginfo(f"Reset alarm count: [{self.motor_reset_alarm_conter}].")
            rospy.loginfo(f"Reset Done.")
        
    def applyControls(self):
        """
        Computes and applyies control signals based on the control mode (velocity vs. torque)
        """
        if(self._speed_control):
            # Send target velocity to the motor controller
            try:
                self.motors.set_rpm(L_rpm=self._target_whl_rpm["l"],R_rpm=self._target_whl_rpm["r"])
                
            except Exception as e:
                rospy.logerr_throttle(1, "[applyControls] Error in setting wheel velocity: %s", e)

    def cmdVelCallback(self, msg):
        sign_x = -1 if msg.linear.x <0 else 1
        sign_w = -1 if msg.angular.z <0 else 1
        
        vx = msg.linear.x
        w = msg.angular.z
        # rospy.loginfo(f"cmd_vel received for motors, vx : {vx}, w:{w}")

        # Initialize final commanded velocities, after applying constraints
        v_d = vx
        w_d = w

        # Limit velocity by acceleration
        current_t = time()
        dt = current_t - self._last_cmd_t
        self._last_cmd_t = current_t
        odom = self._diff_drive.calcRobotOdom(dt)
        current_v = odom['v']
        current_w = odom['w']

        # # Figure out the max acceleration sign
        # dv = vx-current_v
        # abs_dv = abs(dv)
        # if (abs_dv > 0):
        #     lin_acc = (abs_dv/dv)*self._max_lin_accel
        # else:
        #     lin_acc = self._max_lin_accel

        # dw = w-current_w
        # abs_dw = abs(w-current_w)
        # if (abs_dw > 0):
        #     ang_acc = dw/abs_dw * self._max_ang_accel
        # else:
        #     ang_acc = self._max_ang_accel

        # # Maximum acceptable velocity given the acceleration constraints, and current velocity
        # max_v = current_v + dt*lin_acc
        # max_w = current_w + dt*ang_acc

        # # Compute & compare errors to decide whether to scale down the desired velocity
        # # For linear vel
        # ev_d = abs(vx-current_v)
        # ev_max = abs(max_v - current_v)
        # if ev_d > ev_max:
        #     v_d=max_v

        # # For angular vel
        # ew_d = abs(w-current_w)
        # ew_max = abs(max_w - current_w)
        # if ew_d > ew_max:
        #     w_d = max_w

        if (abs(v_d) > self._max_vx):
            rospy.logwarn_throttle(1, "Commanded linear velocity %s is more than maximum magnitude %s", sign_x*vx, sign_x*self._max_vx)
            v_d = sign_x * self._max_vx
        if (abs(w_d) > self._max_w):
            rospy.logwarn_throttle(1, "Commanded angular velocity %s is more than maximum magnitude %s", sign_w*w, sign_w*self._max_w)
            w_d = sign_w * self._max_w

        # Compute wheels velocity commands [rad/s]
        (wl, wr) = self._diff_drive.calcWheelVel(v_d,w_d)

        # convert rad/s to rpm
        wl_rpm = self.rpsToRpm(wl)
        wr_rpm = self.rpsToRpm(wr)
        rospy.loginfo(f"Motor target RPS, L=[{wl}], R=[{wr}]")
        self._target_whl_rpm["l"] = int(wl_rpm * self._flip_direction["l"])
        self._target_whl_rpm["r"] = int(wr_rpm * self._flip_direction["r"])
        rospy.loginfo(f"Motor target RPM, L=[{self._target_whl_rpm['l']}], R=[{self._target_whl_rpm['r']}]")

        # Apply control in the main loop

    def pubOdom(self):
        """ Computes & publishes odometry msg
        """
        try:
            vl, vr = self.motors.get_rpm()
            rospy.loginfo(f"Motor current RPM, L:[{vl}], R:[{vr}]")
            self._diff_drive._l_vel = self.rpmToRps(vl) * self._flip_direction["l"]
            self._diff_drive._r_vel = self.rpmToRps(vr) * self._flip_direction["r"]
            rospy.loginfo(f"Motor current RPS, L:[{self._diff_drive._l_vel}], R:[{self._diff_drive._r_vel}]")

            # resetting motor_reset_alarm_conter
            # if vl > 0 or vr > 0:
            #     rospy.loginfo(f"Reset alarm count:[{self.motor_reset_alarm_conter}] to 0.")
            #     self.motor_reset_alarm_conter = 0
                
        except Exception as e :
            rospy.logerr_throttle(1, " Error in pubOdom: %s. Check driver connection", e)
            #rospy.logerr_throttle(1, "Available nodes = %s", self._network._network.scanner.nodes)

        now = time()

        dt= now - self._last_odom_dt
        self._last_odom_dt = now

        odom = self._diff_drive.calcRobotOdom(dt)

        msg = Odometry()

        time_stamp = rospy.Time.now()
        msg.header.stamp = time_stamp
        msg.header.frame_id=self._odom_frame
        msg.child_frame_id = self._robot_frame

        msg.pose.pose.position.x = odom["x"]
        msg.pose.pose.position.y = odom["y"]
        msg.pose.pose.position.z = 0.0
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, odom['yaw'])
        msg.pose.pose.orientation.x = odom_quat[0]
        msg.pose.pose.orientation.y = odom_quat[1]
        msg.pose.pose.orientation.z = odom_quat[2]
        msg.pose.pose.orientation.w = odom_quat[3]
        # pose covariance
        msg.pose.covariance[0] = 1000.0 # x-x
        msg.pose.covariance[7] = 1000.0 # y-y
        msg.pose.covariance[14] = 1000.0 # z-z
        msg.pose.covariance[21] = 1000.0 # roll
        msg.pose.covariance[28] = 1000.0 # pitch
        msg.pose.covariance[35] = 1000.0 # yaw

        # For twist, velocities are w.r.t base_link. So, only x component (forward vel) is used
        msg.twist.twist.linear.x = odom['v']
        msg.twist.twist.linear.y = 0 #odom['y_dot']
        msg.twist.twist.angular.z = odom['w']
        msg.twist.covariance[0] = 0.1 # vx
        msg.twist.covariance[7] = 0.1 # vx
        msg.twist.covariance[14] = 1000.0 # vz
        msg.twist.covariance[21] = 1000.0 # omega_x
        msg.twist.covariance[28] = 1000.0 # omega_y
        msg.twist.covariance[35] = 0.1 # omega_z
        
        self._odom_pub.publish(msg)
        if self._pub_tf:
            # Send TF
            self._tf_br.sendTransform((odom['x'],odom['y'],0),odom_quat,time_stamp,self._robot_frame,self._odom_frame)

        # msg = Float64()
        # msg.data = odom["v"] # Forward velocity
        # self._vel_pub.publish(msg)

    # def pubMotorState(self):
    #     for t in ["l", "r"]:
    #         msg = State()
    #         msg.header.stamp = rospy.Time.now()
    #         msg.node_id = self._wheel_ids[t]
            
    #         # Voltage
    #         try:
    #             volts_dict = self._network.getVoltage(self._wheel_ids[t])
    #             volts = volts_dict['value']
    #             msg.voltage = volts
    #         except:
    #             pass

    #         # Target current in mA
    #         msg.target_current_mA = self._target_current[t]
    #         # Target current in A
    #         msg.target_current_A = self._target_current[t]/1000.0
            
    #         # Motor current
    #         try:
    #             curr_dict = self._network.getMotorCurrent(self._wheel_ids[t])
    #             curr = curr_dict['value']
    #             msg.current = curr
    #         except:
    #             pass

    #         # Error Code
    #         try:
    #             err_dict = self._network.getErrorCode(self._wheel_ids[t])
    #             code = err_dict['value']
    #             msg.error_code = code
    #         except:
    #             pass

    #         # Current speed, rpm
    #         try:
    #             msg.actual_speed = self._current_whl_rpm[t]
    #         except:
    #             pass

    #         # Target speed, rpm
    #         try:
    #             msg.target_speed = self._target_whl_rpm[t]
    #         except:
    #             pass

    #         self._motor_state_pub_dict[t].publish(msg)


    def mainLoop(self):
        rate = rospy.Rate(self._loop_rate)

        while not rospy.is_shutdown() and self.motor_reset_alarm_conter < 10:
            now = time()

            dt = now - self._last_cmd_t
            if (dt > self._cmd_timeout):
                # set zero velocity
                for t in ["l", "r"]:
                    self._target_whl_rpm[t]=0

            # Apply controls
            self.applyControls()

            # Publish wheel odom
            self.pubOdom()

            self.check_driver_health()
            # Publish Motors state
            # self.pubMotorState()
            
            rate.sleep()
        
        rospy.loginfo("ZLAC8015D | Closed motor node \n")
        # Disable the motors on closing
        self.motors.disable_motor()
  

if __name__ == "__main__":
    rospy.init_node("** Motor driver node started ** \n",anonymous=True, log_level=rospy.INFO)
    try:
        driver = Driver()

        driver.mainLoop()
    except rospy.ROSInterruptException:
        driver.motors.disable_motor()
