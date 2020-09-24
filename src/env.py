#!/usr/bin/env python3

import rospy
from mdmi_game.srv import DroneStatus, DroneStatusResponse
# from std_srvs.srv import Empty, EmptyResponse


class MDMIGameEnvNode(object):
	def __init__(self, nd=1, ni=2):

		self.ni = ni
		self.nd = nd

		# players
		self.defenders = ['D'+str(i) for i in range(nd)]
		self.intruders = ['I'+str(i) for i in range(ni)]


		# service client for individual players, server defined in player node
		self.set_ind_status_service_clients = [self.get_service_client(p) for p in self.defenders+self.intruders]
		# print(self.set_ind_status_service_clients)
		# service server to control all the players, called from terminal
		rospy.Service('set_status_for_all', DroneStatus, self.set_status_for_all_srv_callback)

	def set_status_for_all_srv_callback(self, req):
		for set_status in self.set_ind_status_service_clients:
			resp = set_status(req.status)
			# print('called set status service')
		return DroneStatusResponse(resp.status)
	
	def get_service_client(self, p):
		srv_name = '/crazyflie2_'+str(p)+'/set_status'
		rospy.loginfo('waiting for ' + srv_name + ' Service ...........')
		rospy.wait_for_service(srv_name)
		rospy.loginfo('found ' + srv_name + ' Service')
		return rospy.ServiceProxy(srv_name, DroneStatus)

if __name__ == '__main__':

	rospy.init_node('environment', anonymous=True)
	nd = rospy.get_param("~nd", 2)
	ni = rospy.get_param("~ni", 1)
	# print('!!!!!!!!!!received', nd, ni)
	env = MDMIGameEnvNode(nd=nd, ni=ni)

	# rospy.Timer(rospy.Duration(.1), intruder.iteration)

	rospy.spin()
		
