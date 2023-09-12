
#!/usr/bin/env python3
import rospy
from zlac_driver.ZLAC8015D import Controller
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Float32

def twist_to_diffdrive(linear, angular, coef_linear, coef_angular):
	# Twist goes from -0.5 to 0.5 in both composante
	# For normal speed control (0 to 0.3 m/s) motor input (in RPM) goes from 0 to 33 
	# The coefficient needed for differentes speed are showed next :
	# -----------------------------------------------------------
	# |  m/s  |  km/h  |  RPM  |  coef_linear  |  coef_angular  |
	# -----------------------------------------------------------
	# |  0.3  |    1   |  33   |	   66      |	    66      |		
	# |   1   |   3.6  |  112  |	  224      |	   224      |
	# |  1.5  |    5   |  168  |	  336      |	   336      |
	# |   2   |   7.2  |  224  |	  448      |	   448      |
	# |  2.5  |   10   |  280  |	  560      |	   560      |

	linear_speed = int(linear.x*coef_linear)
	angular_speed = int(angular.z*coef_angular)

	if angular_speed > 0:
		leftrpm = linear_speed - abs(angular_speed)
		rightrpm = linear_speed + abs(angular_speed)
	elif angular_speed < 0:
		leftrpm = linear_speed + abs(angular_speed)
		rightrpm = linear_speed - abs(angular_speed)
	else:
		leftrpm = linear_speed
		rightrpm = linear_speed

	rightrpm = -rightrpm
	
	return [leftrpm,rightrpm]


def cmd_vel_callback(data):
	# Convert Twist to differential driver
	[leftrpm , rightrpm] = twist_to_diffdrive(data.linear, data.angular, coef_linear, coef_angular)

	# Publish command rpm
	cmd_rpm_left.publish(leftrpm)
	cmd_rpm_right.publish(rightrpm)

	#Send data to serial
	motors.set_rpm(leftrpm,rightrpm)
	#motors.set_rpm(-leftrpm,-rightrpm,2)


def motor_init(accel_time, decel_time):
	# Disable motor
	motors.disable_motor()

	# Set acceleration and deceleration time
	motors.set_accel_time(accel_time,accel_time)
	motors.set_decel_time(decel_time,decel_time)

	# Set speed control mode
	motors.set_mode(3)
	motors.enable_motor()



if __name__ == "__main__":
    # Init the ROS node

	rospy.init_node("zlac_driver_control")
	rate = rospy.Rate(100)

	# Getting ROS params
	port = rospy.get_param("~port", "/dev/ttyUSB0")
	coef_linear = rospy.get_param("~coef_linear", 66)
	coef_angular = rospy.get_param("~coef_angular", 66)


	# Creating driver instance
	motors = Controller(port=port)
	# motors = ZLAC8015D.Controller(port="/dev/ttyUSB0")
	# motor_init(1000, 1000)
	motor_init(100, 100)

	# Setup subscriber and publisher
	rospy.loginfo("# ZLAC8015D | Setup subscriber")
	subjoy = rospy.Subscriber("/cmd_vel",Twist,cmd_vel_callback,queue_size=1)

	cmd_rpm_left = rospy.Publisher("/zlac_driver/cmd/front_left", Int16, queue_size=1)
	cmd_rpm_right = rospy.Publisher("/zlac_driver/cmd/front_right", Int16, queue_size=1)

	rpm_left = rospy.Publisher("/zlac_driver/rpm/front_left", Float32, queue_size=1)
	rpm_right = rospy.Publisher("/zlac_driver/rpm/front_right", Float32, queue_size=1)

	# Read encoder value and publish it
	while True:
		try:
			
			# Read RPM from serial RS
			#rpmL, rpmR = motors.get_rpm(1)
			rpmL = 0
			rpmR = rpmL
			# Publsih RPM from serial RS
			rpm_left.publish(round(rpmL,1))
			rpm_right.publish(round(rpmR,1))

		except KeyboardInterrupt:
			motors.disable_motor(1)
			motors.disable_motor(2)
			break

	rate.sleep()