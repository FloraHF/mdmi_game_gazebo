#!/usr/bin/env python3

import numpy as np
from pyllist import dllist

import rospy
from std_msgs.msg import String

from BasePlayer import PlayerNode
from Geometries import DominantRegion
from utils import dist, norm

class DefenderNode(PlayerNode):
	"""docstring for Intruder"""
	def __init__(self, *arg, **kwarg):
		super(DefenderNode, self).__init__(*arg, **kwarg)
		self.ub = np.ceil(self.ni/self.nd)
		self.caplist = []

		# for preferred intruder list
		self.pref_subs = [rospy.Subscriber('/crazyflie2_'+str(p) + '/communication/pref', String, \
							self.get_prefsub_callback(p)) for p in self.team_all]
		self.pref_update = rospy.Timer(rospy.Duration(.1), self.pref_update_callback)

		# publishers
		self.pref_pub = rospy.Publisher('/crazyflie2_'+str(self) + '/communication/pref', String, queue_size=1)

		# log: the targeted intruder
		with open(self.datadir+'/Itarg.csv', 'w') as f:
			f.write('t,i,e,pref\n')

	def get_team(self):
		return ['D'+str(i) for i in range(self.nd) if i != self.id]

	def get_oppo(self):
		return ['I'+str(i) for i in range(self.ni)]
	
	def get_efficiency(self, xi, vd, vi):
		dr = DominantRegion(self.r, vd/vi, xi, [self.state.x], offset=0)
		xw = self.target.deepest_point_in_dr(dr)
		tlevel = self.target.level(xw)
		dlevel = dist(xw, self.state.x)
		return tlevel/dlevel

	def capture_handler(self):
		clear_neib = True
		temp_state_oppo_neigh = {k:v for k, v in self.state_oppo_neigh.items()}

		for i in temp_state_oppo_neigh:
			if self.is_capture(i):
				self.caplist.append(i)
				rospy.loginfo(str(self)+' reports: '+i+' is captured')
			else:
				clear_neib = False

		if clear_neib:
			rospy.loginfo(str(self)+' reports: not seeing any intruders')
			self.status[0] = 'standby'

	def entering_handler(self):
		pass	

	def strategy(self):

		# copy this to prevent being empty again after the following if statement
		pref_dict = {k: v for k, v in self.state.pref.items()} 
		if not pref_dict:
			return np.array([0, 0]) # TODO: return to x0

		# TODO: select current intruder by value
		icurr = next(iter(pref_dict.items()))[0]

		# self.state_oppo_neigh could be changed by other threads
		# likely when icurr captured by other defenders)
		if icurr not in self.state_oppo_neigh:
			return np.array([0, 0]) # TODO: return to x0

		istate = self.state_oppo_neigh[icurr]

		dr = DominantRegion(self.r, .5/.3, istate.x, [self.state.x], offset=0)
		xw = self.target.deepest_point_in_dr(dr)
		dx = xw - self.state.x

		with open(self.datadir+'/Itarg.csv', 'a') as f:
			f.write('%.4f,%s,%.4f,%s\n'%(self.state.t, icurr, pref_dict[icurr], self.prefdict_to_prefstring(pref_dict)))

		return self.vmax*dx/norm(dx)

	def pref_update_callback(self, event):

		# copy to lower the chance of chaning size
		state_oppo_neigh_temp = {k:v for k, v in self.state_oppo_neigh.items()}

		cand_dict = {p: self.get_efficiency(state.x, .5, .3)\
						for p, state in state_oppo_neigh_temp.items()}
		cand_sort = dllist([[k, v] for k, v in sorted(cand_dict.items(), key=lambda x: x[1])])
		pin = 0 if cand_sort.size <= self.ub else int(-self.ub)
		pref = [n.value[0] for n in cand_sort.nodeat(pin).iternext()] if cand_sort.size > 0 else []

		_n = len(pref)
		for i in pref[::-1]:
			# print(str(self), 'before check neigbour', i, pref)
			for d, state in self.state_team_neigh.items():
				if i in state.pref and cand_dict[i] < state.pref[i]:
					# print(str(self), 'before check neigbour', i, pref)
					pref.remove(i)
					break;
					# nremoved += 1
		n_ = len(pref)
		for k in range(_n - n_):
			if cand_sort.nodeat(pin).prev is None:
				break
			pref = [cand_sort.nodeat(pin).prev.value[0]] + pref
			pin -= 1

		pref_dict = {p: cand_dict[p] for p in pref[::-1]}
		self.state.pref = pref_dict

		self.pref_pub.publish(self.prefdict_to_prefstring(pref_dict))
					
	def get_prefsub_callback(self, p):
		def sub_callback(msg):
			pref = self.prefstring_msg_to_prefdict(msg)
			if p in self.state_team_neigh:
				self.state_team_neigh[p].pref = pref

		return sub_callback

	def prefstring_msg_to_prefdict(self, msg):
		pref = dict()
		if msg.data:
			for data in msg.data.split('_'):
				p_e = data.split('=')
				p = p_e[0]
				e = float(p_e[1])
				pref.update({p:e})
		return pref

	def prefdict_to_prefstring(self, pref):
		datalist = [p + '=%.6f'%e for p, e in pref.items()] 
		return '_'.join(list(map(str, datalist)))

	def __repr__(self):
		return 'D' + str(self.id)

if __name__ == '__main__':

	rospy.init_node('defender', anonymous=True)
	resid = rospy.get_param("~res_id", 'res0')
	i = rospy.get_param("~id", 0)
	vmax = rospy.get_param("~vmax", .5)
	x = rospy.get_param("~x", 0)
	y = rospy.get_param("~y", 0)
	r = rospy.get_param("~r", .1)
	nd = rospy.get_param("~nd", 2)
	ni = rospy.get_param("~ni", 1)
	Ro = rospy.get_param("~Ro", 2.)
	Rt = rospy.get_param("~Rt", 5.)

	# print('D'+i, x, y)

	defender = DefenderNode(i, np.array([x, y]), vmax,
							r=r, nd=nd, ni=ni,
							Rteam=Rt, Roppo=Ro,
							resid=resid)

	rospy.Timer(rospy.Duration(.1), defender.iteration)

	rospy.spin()		