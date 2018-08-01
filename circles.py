#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler
from math import floor, cos, atan2, sqrt

vx = 0
vy = 0
vz = 0


def circle_callback(msg):
	global vx, vy, vz
	theta = atan2(y,x)
	vx = -sin(theta)
	vy = cos(theta)
	vz = 0

rospy.init_node('circles', anonymous=True)

swarm_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1000)
swarm_msg = [ModelState() for _ in xrange(25)]
for i in xrange(25):
	swarm_msg[i].model_name = 'quadrotor' + str(i)

dt = 0.1
x = 1
y = 0
z = 0

x_values = np.arange(-1.25, 1.3, 0.005, dtype=None)
x_values_flip = np.arange(1.25, -1.3, -0.005, dtype=None)
y_values = np.sqrt((1.25**2)-(x_values**2))
x_traj = np.concatenate((x_values, x_values_flip))
y_traj = np.concatenate((y_values, -y_values))
n =  len(x_traj)
i = 0
while not rospy.is_shutdown():
	
	x = x_traj[i]
	y = y_traj[i]
	
	for q in xrange(25):

		swarm_msg[q].pose.position.x = x + 3*((q)/5)
		swarm_msg[q].pose.position.y = y + 3*(q%5)
		swarm_msg[q].pose.position.z = 1
		swarm_pub.publish(swarm_msg[q])

	i = i+1

	if i > n-1:
		i = 0




	'''x = x + vx*dt
	y = y + vy*dt
	z = z + vz*dt

	q = quaternion_from_euler(-0.25*vy, 0.25*vx, 0)

	for i in xrange(25):
		swarm_msg[i].pose.position.x = x
		swarm_msg[i].pose.position.y = y 
		swarm_msg[i].pose.position.z = 1

		swarm_msg[i].pose.orientation.x = q[0]
		swarm_msg[i].pose.orientation.y = q[1]
		swarm_msg[i].pose.orientation.z = q[2]
		swarm_msg[i].pose.orientation.w = q[3]'''

	

