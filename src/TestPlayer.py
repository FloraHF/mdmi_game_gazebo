#!/usr/bin/env python3

import numpy as np

import rospy
from mdmi_game.srv import DroneStatus, DroneStatusResponse
from trajectory_msgs.msg import MultiDOFJointTrajectory
from std_msgs.msg import String

from utils import odometry_msg_to_player_state, create_multi_dof_joint_trajectory_msg
# from std_srvs.srv import Empty, EmptyResponse


class TestPlayerNode(object):
	def __init__(self, i, x0):

		self.id = i
		self.x0 = x0
		self.altitude = 1

		self.status = ''

		# self.set_status_service_client = self.get_service_client(self.id)

		# publisher
		self.trajectory_msg_pub = rospy.Publisher('/crazyflie2_'+self.id+'/command/trajectory', MultiDOFJointTrajectory, queue_size=1)
		self.controller_status_pub = rospy.Publisher('/crazyflie2_'+self.id+'/command/controller_status', String, queue_size=1)

		# service
		rospy.Service('/crazyflie2_'+self.id+'/set_status', DroneStatus, self.status_srv_callback)

	# ============ service callbacks ============
	def status_srv_callback(self, req):
		self.status = req.status
		if req.status in ['play', 'land', 'standby']:
			controller_status = 'acceleration_altitude'
		if req.status == 'deploy':
			controller_status = 'waypoint'
		self.controller_status_pub.publish(controller_status)
		# print('seting status for ', str(self), ' as', self.status)
		return DroneStatusResponse(self.status)

	def play(self):
		u = [.1, .1, 0.]

		trajectory_msg = create_multi_dof_joint_trajectory_msg(1)
		trajectory_msg.joint_names[0] = 'acceleration_altitude'

		trajectory_msg.points[0].accelerations[0].linear.x = u[0]
		trajectory_msg.points[0].accelerations[0].linear.y = u[1]
		trajectory_msg.points[0].transforms[0].translation.z = self.altitude

		self.trajectory_msg_pub.publish(trajectory_msg)

	def deploy(self):
		trajectory_msg = create_multi_dof_joint_trajectory_msg(1)

		trajectory_msg.joint_names[0] = 'waypoint'
		trajectory_msg.points[0].transforms[0].translation.x = self.x0[0]
		trajectory_msg.points[0].transforms[0].translation.y = self.x0[1]
		trajectory_msg.points[0].transforms[0].translation.z = self.altitude
		
		self.trajectory_msg_pub.publish(trajectory_msg)

	def land(self):
		trajectory_msg = create_multi_dof_joint_trajectory_msg(1)
		trajectory_msg.joint_names[0] = 'acceleration_altitude'

		self.trajectory_msg_pub.publish(trajectory_msg)

	def iteration(self, event):

		if not rospy.is_shutdown():

			if self.status == 'deploy':
				self.deploy()
				# while self.status == 'deploy':
				# 	pass
			if self.status == 'play':
				self.play()

			if self.status == 'land':
				self.land()
				# while self.status == 'land':
				# 	pass		
if __name__ == '__main__':

	rospy.init_node('test_player', anonymous=True)
	i = rospy.get_param("~id", 0)
	x = rospy.get_param("~x", 0)
	y = rospy.get_param("~y", 0)
		
	player = TestPlayerNode(i, np.array([x, y]))

	rospy.Timer(rospy.Duration(.1), player.iteration)

	rospy.spin()

		
