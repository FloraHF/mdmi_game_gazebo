#!/usr/bin/env python3

import os
import shutil

import numpy as np
import pandas as pd
import rospy

# messages
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory

# services
from mdmi_game.srv import DroneStatus, DroneStatusResponse

# collision avoidance: orca
from pyorca.pyorca import Agent, orca

# for the game
from Geometries import CircleTarget
from utils import norm, dist, PlayerState
from utils import odometry_msg_to_player_state, create_multi_dof_joint_trajectory_msg


class PlayerNode(object):

	def __init__(self, i, x, vmax, ni=1, nd=2, r=1., Rtarg=1.25, Rteam=5., Roppo=5., resid='res0'):
		# environment settings
		self.target = CircleTarget(Rtarg)
		self.ni = ni
		self.nd = nd
		self.t = 0.

		# player settings
		self.altitude = 1.
		self.r = r
		self.id = i
		self.Rt = Rteam
		self.Ro = Roppo

		self.status = ['standby', '']
		self.x0 = x
		self.vmax = vmax
		self.state = PlayerState(self.t, self.x0, self.altitude)

		# id of all the players
		self.team_all = self.get_team()
		self.oppo_all = self.get_oppo()

		# states of neighbours
		self.state_team_neigh = dict()
		self.state_oppo_neigh = dict()

		# subscribers
		self.self_sub = rospy.Subscriber('/crazyflie2_'+str(self)+'/odometry', Odometry, self.selfsub_callback)
		self.team_subs = [rospy.Subscriber('/crazyflie2_'+str(p)+'/odometry', Odometry, \
							self.get_statesub_callback(p, self.state_team_neigh, self.Rt)) for p in self.team_all] 
		self.oppo_subs = [rospy.Subscriber('/crazyflie2_'+str(p)+'/odometry', Odometry, \
							self.get_statesub_callback(p, self.state_oppo_neigh, self.Ro)) for p in self.oppo_all] 

		# publisher
		self.trajectory_msg_pub = rospy.Publisher('/crazyflie2_'+str(self)+'/command/trajectory', MultiDOFJointTrajectory, queue_size=1)
		self.controller_status_pub = rospy.Publisher('/crazyflie2_'+str(self)+'/command/controller_status', String, queue_size=1)

		# service
		rospy.Service('/crazyflie2_'+str(self)+'/set_status', DroneStatus, self.status_srv_callback)

		# create files for data saving, clear existing files if exist
		self.datadir = '/home/flora/mdmi_data/'+resid+'/'+str(self)
		if os.path.exists(self.datadir): 
			shutil.rmtree(self.datadir)
		os.makedirs(self.datadir)

		# record parameters used by the player
		with open(self.datadir + '/param.csv', 'w') as f:
			
			# title
			f.write('param,value\n')

			# target info
			f.write('target:type,%s\n'%str(self.target)) 
			f.write('target:x0,%.2f\n'%self.target.x0)
			f.write('target:y0,%.2f\n'%self.target.y0)
			if 'circle' in str(self.target):
				f.write('target:R,%.2f\n'%self.target.size)

			f.write('nd,%d\n'%self.nd)
			f.write('ni,%d\n'%self.ni)
			
			# player info
			f.write('id,%s\n'%str(self))
			f.write('r,%.2f\n'%self.r)
			f.write('vmax,%.2f\n'%self.vmax) 
			f.write('x0,%.2f\n'%self.x0[0])
			f.write('y0,%.2f\n'%self.x0[1])
			f.write('z,%.2f\n'%self.altitude)
			f.write('Rteam,%.2f\n'%self.Rt)
			f.write('Roppo,%.2f\n'%self.Ro)

	# ============ functions for the game ============
	def get_team(self):
		raise NotImplementedError

	def get_oppo(self):
		raise NotImplementedError

	def strategy(self):
		raise NotImplementedError

	def capture_handler(self):
		raise NotImplementedError

	def entering_handler(self):
		raise NotImplementedError

	def is_capture(self, oppo):
		if oppo not in self.state_oppo_neigh:
			return False
		return dist(self.state.x, self.state_oppo_neigh[oppo].x) < self.r

	# ============ actions for different status ============
	def standby(self):
		trajectory_msg = create_multi_dof_joint_trajectory_msg(1)
		trajectory_msg.points[0].transforms[0].translation.z = self.altitude

		self.trajectory_msg_pub.publish(trajectory_msg)

	def win(self):
		trajectory_msg = create_multi_dof_joint_trajectory_msg(1)
		trajectory_msg.points[0].transforms[0].translation.z = self.altitude+.3

		self.trajectory_msg_pub.publish(trajectory_msg)

	def play(self):
		u_pref = self.strategy()
		orca_self = Agent(self.state.x, self.state.v, .1, u_pref)
		orca_other = []

		for other, state in self.state_team_neigh.items():
			orca_other.append(Agent(state.x, state.v, .1, (0, 0)))

		u, _ = orca(orca_self, orca_other, 2., 1e-2)
		
		# print(str(self), u_pref, u)

		trajectory_msg = create_multi_dof_joint_trajectory_msg(1)

		trajectory_msg.points[0].velocities[0].linear.x = u[0]
		trajectory_msg.points[0].velocities[0].linear.y = u[1]
		trajectory_msg.points[0].transforms[0].translation.z = self.altitude

		# self.capture_handler(*arg)

		self.trajectory_msg_pub.publish(trajectory_msg)

	def deploy(self):
		trajectory_msg = create_multi_dof_joint_trajectory_msg(1)

		trajectory_msg.points[0].transforms[0].translation.x = self.x0[0]
		trajectory_msg.points[0].transforms[0].translation.y = self.x0[1]
		trajectory_msg.points[0].transforms[0].translation.z = self.altitude
		
		self.trajectory_msg_pub.publish(trajectory_msg)

	def land(self):
		trajectory_msg = create_multi_dof_joint_trajectory_msg(1)

		self.trajectory_msg_pub.publish(trajectory_msg)

	# ============ service callbacks ============
	def status_srv_callback(self, req):
		self.status[0] = req.status
		if req.status in ['play', 'land', 'standby']:
			controller_status = 'acceleration_altitude'
		if req.status == 'deploy':
			controller_status = 'waypoint'
		self.controller_status_pub.publish(controller_status)
		# print('seting status for ', str(self), ' as', self.status)
		return DroneStatusResponse(self.status[0])

	# ============ subscriber callbacks ============
	def selfsub_callback(self, msg):
		state = odometry_msg_to_player_state(msg)
		self.state.update_xzv(t=state.t, x=state.x, z=state.z, v=state.v)

	def get_statesub_callback(self, p, pset, R):
		def sub_callback(msg):
			state = odometry_msg_to_player_state(msg)
			if dist(state.x, self.state.x) <= R and state.z > 0.5:
				pset.update({p: state})
			else:
				pset.pop(p, '')
		return sub_callback	

	# ============ main iteration ============
	def iteration(self, event):

		if not rospy.is_shutdown():

			if self.status[0] == 'play':
				self.play()
				self.capture_handler()
				self.entering_handler()

			if self.status[0] == 'standby':
				self.standby()

			if self.status[0] == 'win':
				self.win()

			if self.status[0] == 'deploy':
				self.deploy()

			if self.status[0] == 'land':
				self.land()

	# ============ class functions ============
	def __repr__(self):
		return ''
