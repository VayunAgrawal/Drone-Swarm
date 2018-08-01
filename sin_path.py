#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler
from math import floor, cos, atan2, sqrt, pi, sin
import time
import sys, termios, tty, os, time

vx = 0
vy = 0
vz = 0
state = 0
'''x_original[25]
y_original[25]
z_original[25]'''


def joy_callback(msg):
	'''global vx, vy, vz
	theta = atan2(y,x)
	vx = -sin(theta)
	vy = cos(theta)
	vz = 0'''
	global state
	if msg.buttons[0]:
		if state < 5:
			state += 1
			time.sleep(7)
		elif state == 5:
			state = 0
			time.sleep(7)

rospy.init_node('sin_path', anonymous=True)

swarm_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1000)
container_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

rospy.Subscriber('/joy', Joy, joy_callback)

container_msg = ModelState()
container_msg.model_name = 'box'
container_msg.pose.position.x = 0
container_msg.pose.position.y = 0
container_msg.pose.position.z = 0.5

swarm_msg = [ModelState() for _ in xrange(25)]
for i in xrange(25):
	swarm_msg[i].model_name = 'quadrotor' + str(i)

dt = 0.1
x = 0
y = 0
z = 0

x_values = np.arange(-5, 5, 0.0009, dtype=None)
x_values_flip = np.arange(5, -5, -0.0009, dtype=None)
y_values = 5*np.sin(2*np.pi*x_values)
x_traj = np.concatenate((x_values, x_values_flip))
y_traj = np.concatenate((y_values, -y_values))
x_square_pos = [0,3,3,3,3,2,1,0,-1,-2,-3,-3,-3,-3,-3,-3,-3,-2,-1,0,1,2,3,3,3]
y_square_pos = [0,0,1,2,3,3,3,3,3,3,3,2,1,0,-1,-2,-3,-3,-3,-3,-3,-3,-3,-2,-1]
x_square_pos = np.subtract(x_square_pos,4)
y_square_pos = np.subtract(y_square_pos,4)
n =  len(x_traj)
i = 0
pos_counter = 0
while not rospy.is_shutdown():

	if state == 0:
		x = x_traj[i]
		y = y_traj[i]
		
		m = quaternion_from_euler(0.25*cos(2*pi*x), 0.25*sin(2*pi*x), 0)


		for q in xrange(25):

			swarm_msg[q].pose.position.x = x + 10*((q)/5)
			swarm_msg[q].pose.position.y = y + 10*(q%5)
			swarm_msg[q].pose.position.z = 5
			if i <= n/2:
				swarm_msg[q].pose.orientation.x = -m[0]
				swarm_msg[q].pose.orientation.y = -m[1]
				swarm_msg[q].pose.orientation.z = m[2]
				swarm_msg[q].pose.orientation.w = m[3]
			else:
				swarm_msg[q].pose.orientation.x = m[0]
				swarm_msg[q].pose.orientation.y = m[1]
				swarm_msg[q].pose.orientation.z = m[2]
				swarm_msg[q].pose.orientation.w = m[3]


			swarm_pub.publish(swarm_msg[q])

		i = i+1

		if i > n-1:
			i = 0

	if state == 1:

		for q in xrange(25):

			'''if swarm_msg[q].pose.position.x != x_square_pos[q] or swarm_msg[q].pose.position.y != y_square_pos[q]:'''

			x = swarm_msg[q].pose.position.x
			y = swarm_msg[q].pose.position.y
			z = swarm_msg[q].pose.position.z

			x_waypoint = 0
			y_waypoint = 0
			z_waypoint = q

			x_goal = x_square_pos[q]
			y_goal = y_square_pos[q]
			# theta_diff = ang_diff(theta_start, atan2(y_diff, x_diff))
			x_diff = x_goal-x
			y_diff = y_goal-y

			'''traj = TrajectoryGenerator([x_original[q], y_original[q], z_original[q]], [0, 0, q], T, start_vel = [0,0,0], des_vel = [1,0,0])
			traj.solve()
			x_c = traj.x_c
			y_c = traj.y_c
			z_c = traj.z_c'''
			theta = atan2(y_diff, x_diff)

			'''x = x_c[0,0]*t**5 + x_c[1,0]*t**4 + x_c[2,0]*t**3 + x_c[3,0]*t**2 + x_c[4,0]*t + x_c[5,0]
			y = y_c[0,0]*t**5 + y_c[1,0]*t**4 + y_c[2,0]*t**3 + y_c[3,0]*t**2 + y_c[4,0]*t + y_c[5,0]
			z = z_c[0,0]*t**5 + z_c[1,0]*t**4 + z_c[2,0]*t**3 + z_c[3,0]*t**2 + z_c[4,0]*t + z_c[5,0]'''
			distance = sqrt(x_diff**2 + y_diff**2)
			v = .04*distance
			# v_theta = Kh*theta_diff
			v_x = v*cos(theta)
			v_y = v*sin(theta)
			x = x + v_x*dt
			y = y + v_y*dt
			#theta = theta + Kh*ang_diff(theta_goal, theta)*dt
			#vehicle.update_pose(x, y, theta)
			m = quaternion_from_euler(v_y, v_x, 0)

			swarm_msg[q].pose.position.x = x
			swarm_msg[q].pose.position.y = y
			swarm_msg[q].pose.position.z = z
			swarm_msg[q].pose.orientation.x = m[0]
			swarm_msg[q].pose.orientation.y = m[1]
			swarm_msg[q].pose.orientation.z = m[2]
			swarm_msg[q].pose.orientation.w = m[3]
			swarm_pub.publish(swarm_msg[q])
	
	if state == 2:
		for q in xrange(25):
			if swarm_msg[q].pose.position.z > 2:
				swarm_msg[q].pose.position.z -= 0.01
				swarm_pub.publish(swarm_msg[q])
			else:
				swarm_msg[q].pose.position.z = 2
				swarm_pub.publish(swarm_msg[q])

	if state == 3:
		'''if container_msg.pose.position.z < 3:
			container_msg.pose.position.z += 0.005
			container_msg.pose.position.x = -4
			container_msg.pose.position.y = -4
		else:
			container_msg.pose.position.z = 3
			container_msg.pose.position.x = -4
			container_msg.pose.position.y = -4
		container_pub.publish(container_msg)'''

		for q in xrange(25):
			if swarm_msg[q].pose.position.z<5:
				swarm_msg[q].pose.position.z += 0.01
			else:
				swarm_msg[q].pose.position.z = 5
			swarm_pub.publish(swarm_msg[q])

		container_msg.pose.position.z = swarm_msg[q].pose.position.z - 2
		container_pub.publish(container_msg)
	if state == 4:

		for q in xrange(25):

			x = swarm_msg[q].pose.position.x
			y = swarm_msg[q].pose.position.y

			x_goal = x_square_pos[q] + 25
			y_goal = y_square_pos[q] + 25
			# theta_diff = ang_diff(theta_start, atan2(y_diff, x_diff))
			x_diff = x_goal-x
			y_diff = y_goal-y
			theta = atan2(y_diff, x_diff)
			
			distance = sqrt(x_diff**2 + y_diff**2)
			v = .03*distance
			# v_theta = Kh*theta_diff
			v_x = v*cos(theta)
			v_y = v*sin(theta)
			x = x + v_x*dt
			y = y + v_y*dt
			#theta = theta + Kh*ang_diff(theta_goal, theta)*dt
			#vehicle.update_pose(x, y, theta)
			
			m = quaternion_from_euler(v_y, v_x, 0)

			swarm_msg[q].pose.position.x = x
			swarm_msg[q].pose.position.y = y
			swarm_msg[q].pose.position.z = 5
			swarm_msg[q].pose.orientation.x = m[0]
			swarm_msg[q].pose.orientation.y = m[1]
			swarm_msg[q].pose.orientation.z = m[2]
			swarm_msg[q].pose.orientation.w = m[3]
			swarm_pub.publish(swarm_msg[q])

			container_msg.pose.position.x = swarm_msg[0].pose.position.x + 4
			container_msg.pose.position.y = swarm_msg[0].pose.position.y + 4
			container_msg.pose.position.z = 3
			container_pub.publish(container_msg)

	if state == 5:
		for q in xrange(25):

			x = swarm_msg[q].pose.position.x
			y = swarm_msg[q].pose.position.y

			x_goal = x_traj[0] + 10*((q)/5)
			y_goal = y_traj[0] + 10*(q%5)
			# theta_diff = ang_diff(theta_start, atan2(y_diff, x_diff))
			x_diff = x_goal-x
			y_diff = y_goal-y
			theta = atan2(y_diff, x_diff)
			
			distance = sqrt(x_diff**2 + y_diff**2)
			v = .03*distance
			# v_theta = Kh*theta_diff
			v_x = v*cos(theta)
			v_y = v*sin(theta)
			x = x + v_x*dt
			y = y + v_y*dt
			#theta = theta + Kh*ang_diff(theta_goal, theta)*dt
			#vehicle.update_pose(x, y, theta)
			m = quaternion_from_euler(v_y, v_x, 0)

			swarm_msg[q].pose.position.x = x
			swarm_msg[q].pose.position.y = y
			swarm_msg[q].pose.position.z = 5
			swarm_msg[q].pose.orientation.x = m[0]
			swarm_msg[q].pose.orientation.y = m[1]
			swarm_msg[q].pose.orientation.z = m[2]
			swarm_msg[q].pose.orientation.w = m[3]
			swarm_pub.publish(swarm_msg[q])


