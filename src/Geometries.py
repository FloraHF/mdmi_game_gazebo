import numpy as np

from utils import norm, dist
from scipy.optimize import NonlinearConstraint, minimize

class DominantRegion(object):
	def __init__(self, rs, vi, vds, xi, xds, offset=0):
		self.rs = rs
		self.vi = vi
		self.vds = vds
		self.xi = xi
		self.xds = xds
		self.offset = offset

	def __str__(self):
		return 'dr'

	def get_data(self, k=5, n=60):
		x = np.linspace(self.xi[0]-k, self.xi[0]+k, n)
		y = np.linspace(self.xi[0]-k, self.xi[0]+k, n)
		X, Y = np.meshgrid(x, y)
		D = np.zeros(np.shape(X))
		for i, (xx, yy) in enumerate(zip(X, Y)):
			for j, (xxx, yyy) in enumerate(zip(xx, yy)):
				D[i,j] = self.level(np.array([xxx, yyy]))
		return X, Y, D		

	def level(self, x):
		# offset: the distance the defender travels
		for i, (vd, xd, r) in enumerate(zip(self.vds, self.xds, self.rs)):
			if i == 0:
				inDR = (vd/self.vi)*dist(x, self.xi) - self.offset - (dist(x, xd) - r)
			else:
				inDR = max(inDR, (vd/self.vi)*dist(x, self.xi) - self.offset - (dist(x, xd) - r))
		return inDR	
	
	def intersect(self, dr):
		for vd in dr.vds:
			self.vds.append(vd)
		for xd in dr.xds:
			self.xds.append(xd)
		for r in dr.rs:
			self.rs.append(r)

class LineTarget(object):
	"""docstring for LineTarget"""
	def __init__(self, x0=0.0, y0=0, minlevel=-5.):
		self.x0 = x0
		self.y0 = y0
		self.type = 'line'
		self.minlevel = minlevel # cut off -inf to minlevel

	def __str__(self):
		return 'line_%.2f'%self.minlevel

	def level(self, x):
		return x[1] - self.y0

	def deepest_point_in_dr(self, dr, target=None):
		if target is not None:
			def obj(x):
				return max(dr.level(x), -target.level(x))
		else:
			def obj(x):
				return dr.level(x)
		in_dr = NonlinearConstraint(obj, -np.inf, 0)
		sol = minimize(self.level, dr.xi, constraints=(in_dr,))
		return sol.x

class CircleTarget(object):
	def __init__(self, R, x=5.0, y=2.5):
		super(CircleTarget, self).__init__()
		# self.state.p_pos = np.array([x, y])
		self.x0 = x
		self.y0 = y
		self.size = R
		self.type = 'circle'
		self.minlevel = -R

	def __str__(self):
		return 'circle_%.2f'%self.size

	def level(self, x):
		return dist(np.array([self.x0, self.y0]), x) - self.size
		# return sqrt((x[0]-self.x0)**2 + (x[1]-self.y0)**2) - self.R	

	def deepest_point_in_dr(self, dr, target=None):
		if target is not None:
			def obj(x):
				return max(dr.level(x), -target.level(x))
		else:
			def obj(x):
				return dr.level(x)
		in_dr = NonlinearConstraint(obj, -np.inf, 0)
		sol = minimize(self.level, dr.xi, constraints=(in_dr,))
		return sol.x