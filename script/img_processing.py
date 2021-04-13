import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.patches import Ellipse


from math import sqrt, atan2
from scipy import linalg

from get_ellipse import mvee, la


LL = 0.092 + 0.046*np.sqrt(2)/2
fx = fy = 190.68123344056778
K = np.array([[fx, 0.0, 160.5], 
			  [0.0, fy, 160.5], 
			  [0.0, 0.0, 1.0]])


# print(la.inv(K).dot(np.array([170, 160.5, 1])))

img = cv.imread('test_rose.png')
img = cv.imread('test_gzbimage.png')

# img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

edges = cv.Canny(img, 100,200)
contours, hierachy = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
points = np.concatenate(contours).squeeze()

# plt.imshow(edges)
# plt.show()

A, c = mvee(points)

eig, R = linalg.eig(A)
lmd_1 = np.real(eig[0])
lmd_2 = np.real(eig[1])

if lmd_1 > lmd_2: # put the first element as the major axis
	print('switching a b!!')
	eig = np.array([lmd_2, lmd_1])
	R = np.array([[R[1][1], R[0][1]],
				  [R[1][0], R[0][0]]])
else:
	eig = np.array([lmd_1, lmd_2])

a, b = 1./np.sqrt(eig)
alpha = atan2(-R[0][1], R[0][0])*180/np.pi
# print(a, b, alpha)

# x1 = R.dot(np.array([ a, 0])) + c
# x2 = R.dot(np.array([-a, 0])) + c
# y1 = R.dot(np.array([ 0, b])) + c
# y2 = R.dot(np.array([ 0, -b])) + c

# # fig = plt.figure()
# plt.gca().plot(x1[0], x1[1], 'o')
# plt.gca().plot(x2[0], x2[1], 'o')
# plt.gca().plot(y1[0], y1[1], 'o')
# plt.gca().plot(y2[0], y2[1], 'o')
# # ax = fig.add_subplot(111, aspect='equal')

# ellipse2 = Ellipse(xy=c, width=2*a, height=2*b, edgecolor='k',
#     angle=alpha, fc='None', lw=2)
# plt.gca().add_patch(ellipse2)
# plt.axis('equal')

# plt.show()

d = fx*LL/(2*a)
offset = la.inv(K).dot(np.array([c[0], c[1], 1]))
offset = offset[:2]*d
dx = np.array([d, offset[0], offset[1]])
print(dx)