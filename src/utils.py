import numpy as np

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist


# ===== geometry calculations
def norm(x):
	return np.sqrt(sum([xx**2 for xx in x]))
def dist(x, y):
	return norm(x-y)

# ===== game definitions
class PlayerState(object):
 	"""docstring for PlayerState"""
 	def __init__(self, t, x, z, v=np.zeros(2)):
 		self.t = t
 		self.x = np.array([xx for xx in x])
 		self.v = np.array([vv for vv in v])
 		self.z = z
 		# self.a = a
 		self.speed = norm(v)
 		# self.pqr = pqr
 		# self.dpqr = dpqr
 		self.pref = dict()

 	def update_xzv(self, t=None, x=None, z=None, v=None):
 		if t is not None:
 			self.t = t
 		if x is not None:
 			self.x = x
 		if z is not None:
 			self.z = z
 		if v is not None:
 			self.v = v
 		self.speed = norm(v)

# ===== message conversions
def odometry_msg_to_player_state(msg):
	t = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
	x = msg.pose.pose.position
	v = msg.twist.twist.linear
	z = x.z
	x = np.array([x.x, x.y])
	v = np.array([v.x, v.y])
	# a = msg.linear_acceleration[:2]
	# pqr = msg.angular_velocity
	# dpqr = msg.angular_acceleration
	return PlayerState(t, x, z, v)

def create_multi_dof_joint_trajectory_msg(npts):
	trajectory_msg = MultiDOFJointTrajectory()

	trajectory_point = MultiDOFJointTrajectoryPoint()
	trajectory_point.transforms.append(Transform())
	trajectory_point.velocities.append(Twist())
	trajectory_point.accelerations.append(Twist())

	for i in range(npts):
		trajectory_msg.points.append(trajectory_point)
		trajectory_msg.joint_names.append('')

	return trajectory_msg