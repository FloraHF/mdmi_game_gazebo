#!/usr/bin/env python3

import numpy as np
from pyllist import dllist
import itertools

import rospy
from std_msgs.msg import String

from BasePlayer import PlayerNode
from Geometries import DominantRegion
from utils import dist, norm

class DefenderNode(PlayerNode):
	"""docstring for Intruder"""
	def __init__(self, *arg, iselect_mode='value', **kwarg):
		super(DefenderNode, self).__init__(*arg, **kwarg)
		self.ub = np.ceil(self.ni/self.nd)
		self.iselect_mode = iselect_mode
		self.iprev = None
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

		with open(self.datadir + '/param.csv', 'a') as f:
			f.write('iselect_mode,%s\n'%str(self.iselect_mode))

	def get_team(self):
		return ['D'+str(i) for i in range(self.nd) if i != self.id]

	def get_oppo(self):
		return ['I'+str(i) for i in range(self.ni)]
	
	def get_efficiency(self, xi, vd, vi):
		dr = DominantRegion([self.r], vi, [vd], xi, [self.state.x], offset=0)
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
				rospy.loginfo_once(str(self)+' reports: '+i+' is captured')
			else:
				clear_neib = False

		if clear_neib:
			rospy.loginfo_once(str(self)+' reports: not seeing any intruders')
			self.status[0] = 'standby'

	def entering_handler(self):
		pass	

	def value_order(self, iset, iorder):
		
		def recurse(xis, vis, actives, xd):
			if sum(actives) == 0:
				return xis, vis, actives, xd

			for i in range(len(xis)):
				if actives[i]:
					dr = DominantRegion([self.r], vis[i], [self.vmax], xis[i], [xd], offset=0)
					xw = self.target.deepest_point_in_dr(dr)
					dt = dist(xw, xis[i])/vis[i]
					xd = xd + dt*self.vmax*(xw - xd)/dist(xw, xd)
					# xis[i] = np.array([x for x in xw])
					xis[i] = xw
					actives[i] = 0
					break
			for j in range(i+1,len(xis)):
				if actives[j]:
					dr = DominantRegion([self.r], vis[j], [self.vmax], xis[j], [xd], offset=self.vmax*dt)
					xw = self.target.deepest_point_in_dr(dr)
					e = (xw - xis[j])/dist(xw, xis[j])
					xi = xis[j] + e*vis[j]*dt
					# xis[j] = np.array([x for x in xi])
					xis[j] = xi
			return recurse(xis, vis, actives, xd)

		xis, vis, actives = [], [], []
		for i in iorder:
			xis.append(np.array([x for x in iset[i]['s'].x]))
			vis.append(iset[i]['s'].speed)
			actives.append(True)

		xd = np.array([x for x in self.state.x])

		xis, _, _, _ = recurse(xis, vis, actives, xd)

		return min([self.target.level(xw) for xw in xis])

	def strategy(self, iselect_mode='emin'):
		# iselect_mode = value, emin, emax

		# copy this to prevent being empty again after the following if statement
		# pref_dict = {k: v for k, v in self.state.pref.items()} 
		pref_dict = dict()
		for p in self.state.pref:
			if p in self.state_oppo_neigh:
				pref_dict.update({p:{'s':self.state_oppo_neigh[p], 'e':self.state.pref[p]}})

		if not pref_dict:
			return np.array([0, 0]) # TODO: return to x0
		n_pref = len(pref_dict)

		
		# select the current intruder by value
		# if self.iselect_mode == 'value':
		if n_pref < 4: # value
			orders = [order for order in itertools.permutations(pref_dict)]
			values = [self.value_order(pref_dict, order) for order in orders]
			icurr = orders[values.index(max(values))][0]

		# elif self.iselect_mode == 'emin':
		else: # emin
			if self.iprev not in pref_dict: # include self.iprev == None
				icurr = [p for p in pref_dict][-1]
				self.iprev = icurr
			else:
				icurr = self.iprev

		# elif self.iselect_mode == 'emax':
		# 	if self.iprev not in pref_dict: # include self.iprev == None
		# 		icurr = [p for p in pref_dict][0]
		# 		self.iprev = icurr
		# 	else:
		# 		icurr = self.iprev

		# self.state_oppo_neigh could be changed by other threads
		# likely when icurr captured by other defenders
		if icurr not in self.state_oppo_neigh:
			return np.array([0, 0]) # TODO: return to x0

		istate = self.state_oppo_neigh[icurr]

		dr = DominantRegion([self.r], istate.speed, [self.vmax], istate.x, [self.state.x], offset=0)
		xw = self.target.deepest_point_in_dr(dr)
		dx = xw - self.state.x

		with open(self.datadir+'/Itarg.csv', 'a') as f:
			f.write('%.4f,%s,%.4f,%s\n'%(self.state.t, icurr, pref_dict[icurr]['e'], 
										'_'.join(list(map(str, [p + '=%.6f'%d['e'] for p, d in pref_dict.items()])))))
			# f.write('%.4f,%s,%.4f,%s\n'%(self.state.t, icurr, self.state.pref_dict[icurr], self.prefdict_to_prefstring(pref_dict)))

		return self.vmax*dx/norm(dx)

	def pref_update_callback(self, event):

		# copy to lower the chance of chaning size
		state_oppo_neigh_temp = {k:v for k, v in self.state_oppo_neigh.items()}

		cand_dict = {p: self.get_efficiency(state.x, self.vmax, state.speed)\
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

		self.pref_pub.publish('_'.join(list(map(str, [p + '=%.6f'%e for p, e in pref_dict.items()]))))
					
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

	# def prefdict_to_prefstring(self, pref):
	# 	datalist = [p + '=%.6f'%e for p, e in pref.items()] 
	# 	return '_'.join(list(map(str, datalist)))

	def __repr__(self):
		return 'D' + str(self.id)

if __name__ == '__main__':

	rospy.init_node('defender', anonymous=True)

	i = rospy.get_param("~id", 0)
	resid = rospy.get_param("~res_id", 'res0')

	vmax = rospy.get_param("~vmax", .5)
	x = rospy.get_param("~x", 0)
	y = rospy.get_param("~y", 0)

	r = rospy.get_param("~r", .1)
	nd = rospy.get_param("~nd", 2)
	ni = rospy.get_param("~ni", 1)
	Ro = rospy.get_param("~Ro", 2.)
	Rt = rospy.get_param("~Rt", 5.)

	iselect_mode = rospy.get_param("~iselect_mode", 'value')
	# print('D'+i, x, y)

	defender = DefenderNode(i, np.array([x, y]), vmax,
							r=r, nd=nd, ni=ni,
							Rteam=Rt, Roppo=Ro,
							resid=resid,
							iselect_mode=iselect_mode)

	rospy.Timer(rospy.Duration(.1), defender.iteration)

	rospy.spin()		