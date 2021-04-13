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

		# log: the defender capturing, and the time entering the target
		with open(self.datadir+'/Dcap.csv', 'w') as f:
			f.write('t,d\n')
		with open(self.datadir+'/Tent.csv', 'w') as f:
			f.write('t,enter\n')		
	
	def get_team(self):
		return ['I'+str(i) for i in range(self.ni) if i != self.id]

	def get_oppo(self):
		return ['D'+str(i) for i in range(self.nd)]

	def capture_handler(self):
		for d in range(self.nd):
			D = 'D'+str(d)
			if self.is_capture(D):
				self.status[1] = 'captured'
				self.status[0] = 'land'
				rospy.loginfo(str(self)+' reports: captured')
				with open(self.datadir+'/Dcap.csv', 'a') as f:
					f.write('%.4f,%s\n'%(self.state.t, D))
				break

	def entering_handler(self):
		if self.target.level(self.state.x) < -0.01:
			self.status[1] = 'entered'
			self.status[0] = 'win'
			rospy.loginfo(str(self)+' reports: entered the target')
			with open(self.datadir+'/Tent.csv', 'a') as f:
				f.write('%.4f,%d\n'%(self.state.t, 1))	

	def strategy(self):

		# copy to prevent change during iteration
		oppo_dict = {k: v for k, v in self.state_oppo_neigh.items()}

		xds, vds = [], []
		for d, state in oppo_dict.items():
			xds.append(np.array([x for x in state.x]))
			vds.append(state.speed)
		# print(str(self), vs, vd)

		if xds: # TODO: to add velocity estimation
			# dr = DominantRegion(self.env.target.size, vd/norm(self.state.v), self.state.x, xds, offset=0)
			dr = DominantRegion([self.r]*len(xds), self.vmax, vds, self.state.x, xds, offset=0)
			xw = self.target.deepest_point_in_dr(dr)
			dx = xw - self.state.x
			dist = norm(dx)
			if dist > 1e-6:
				return self.vmax*dx/dist

		dx = np.array([self.target.x0, self.target.y0]) - self.state.x
		return vmax*dx/norm(dx)

	def __repr__(self):
		return 'I' + str(self.id)

if __name__ == '__main__':

	rospy.init_node('intruder', anonymous=True)
	resid = rospy.get_param("~res_id", 'res0')
	i = rospy.get_param("~id", 0)

	vmax = rospy.get_param("~vmax", .3)
	x = rospy.get_param("~x", 0)
	y = rospy.get_param("~y", 0)
	z = rospy.get_param("~z", 1.)

	dx = rospy.get_param("~takeoff_offset_x", 0.)
	dy = rospy.get_param("~takeoff_offset_y", 0.)

	r = rospy.get_param("~r", .1)
	nd = rospy.get_param("~nd", 2)
	ni = rospy.get_param("~ni", 1)
	Ro = rospy.get_param("~Ro", 2.)
	Rt = rospy.get_param("~Rt", 5.)
		
	intruder = IntruderNode(i, np.array([x, y]), vmax,
							z=z, offset=np.array([dx, dy]),
							r=r, nd=nd, ni=ni,
							Rteam=Rt, Roppo=Ro,
							resid=resid)

	rospy.Timer(rospy.Duration(.1), intruder.iteration)

	rospy.spin()
