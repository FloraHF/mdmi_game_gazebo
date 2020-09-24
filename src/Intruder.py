#!/usr/bin/env python3

import rospy
import numpy as np

from BasePlayer import PlayerNode
from Geometries import DominantRegion
from utils import dist, norm

class IntruderNode(PlayerNode):
	"""docstring for Intruder"""
	def __init__(self, *arg, **kwarg):
		super(IntruderNode, self).__init__(*arg, **kwarg)
	
	def get_team(self):
		return ['I'+str(i) for i in range(self.ni) if i != self.id]

	def get_oppo(self):
		return ['D'+str(i) for i in range(self.nd)]

	def capture_handler(self):
		for d in range(self.nd):
			if self.is_capture('D'+str(d)):
				self.status[1] = 'captured'
				self.status[0] = 'land'
				rospy.loginfo(str(self)+' reports: captured')
				break

	def entering_handler(self):
		if self.target.level(self.state.x) < -0.01:
			self.status[1] = 'entered'
			self.status[0] = 'standby'
			rospy.loginfo(str(self)+' reports: entered the target')

	def strategy(self):

		# copy to prevent change during iteration
		oppo_dict = {k: v for k, v in self.state_oppo_neigh.items()}

		xds, vs = [], []
		for d, state in oppo_dict.items():
			xds.append(np.array([x for x in state.x]))
			# vs.append(state.speed)
		# vd = np.average(vs)

		if xds: # TODO: to add velocity estimation
			# dr = DominantRegion(self.env.target.size, vd/norm(self.state.v), self.state.x, xds, offset=0)
			dr = DominantRegion(self.r, .5/.3, self.state.x, xds, offset=0)
			xw = self.target.deepest_point_in_dr(dr)
			dx = xw - self.state.x
			dist = norm(dx)
			if dist > 1e-6:
				return 0.15*dx/dist

		return np.zeros(2)

	def get_teamstate_callback(self, p):
		pass

	def get_oppostate_callback(self, p):

		def sub_callback(msg):
			s = self.donestate_msg_to_state(msg)
			if dist(s.x, self.x.x) < self.Ro:
				self.state_oppo_neigh.update({p: s})
		return sub_callback	

	def __repr__(self):
		return 'I' + str(self.id)

if __name__ == '__main__':

	rospy.init_node('intruder', anonymous=True)
	i = rospy.get_param("~id", 0)
	x = rospy.get_param("~x", 0)
	y = rospy.get_param("~y", 0)
	r = rospy.get_param("~r", .1)
	nd = rospy.get_param("~nd", 2)
	ni = rospy.get_param("~ni", 1)
		
	intruder = IntruderNode(i, np.array([x, y]), 
							r=r, nd=nd, ni=ni)

	rospy.Timer(rospy.Duration(.1), intruder.iteration)

	rospy.spin()
