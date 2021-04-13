import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
# import time


# http://stackoverflow.com/questions/14016898/port-matlab-bounding-ellipsoid-code-to-python
# http://stackoverflow.com/questions/1768197/bounding-ellipse/1768440#1768440
# https://minillinim.github.io/GroopM/dev_docs/groopm.ellipsoid-pysrc.html
# https://gist.github.com/Gabriel-p/4ddd31422a88e7cdf953

# def mvee(points, tol=0.0001):
#     """
#     Finds the ellipse equation in "center form"
#     (x-c).T * A * (x-c) = 1
#     """
#     N, d = points.shape
#     Q = np.column_stack((points, np.ones(N))).T
#     err = tol+1.0
#     u = np.ones(N)/N
#     while err > tol:
#         # assert u.sum() == 1 # invariant
#         X = np.dot(np.dot(Q, np.diag(u)), Q.T)
#         M = np.diag(np.dot(np.dot(Q.T, la.inv(X)), Q))
#         jdx = np.argmax(M)
#         step_size = (M[jdx]-d-1.0)/((d+1)*(M[jdx]-1.0))
#         new_u = (1-step_size)*u
#         new_u[jdx] += step_size
#         err = la.norm(new_u-u)
#         u = new_u
#     c = np.dot(u, points)
#     A = la.inv(np.dot(np.dot(points.T, np.diag(u)), points)
#                - np.multiply.outer(c, c))/d
#     return A, c


def mvee(points, tol=0.001):
    """
    Find the minimum volume ellipse.
    Return A, c where the equation for the ellipse given in "center form" is
    (x-c).T * A * (x-c) = 1
    """
    points = np.asmatrix(points)
    N, d = points.shape
    Q = np.column_stack((points, np.ones(N))).T
    err = tol+1.0
    u = np.ones(N)/N
    while err > tol:
        # assert u.sum() == 1 # invariant
        X = Q * np.diag(u) * Q.T
        M = np.diag(Q.T * la.inv(X) * Q)
        jdx = np.argmax(M)
        step_size = (M[jdx]-d-1.0)/((d+1)*(M[jdx]-1.0))
        new_u = (1-step_size)*u
        new_u[jdx] += step_size
        err = la.norm(new_u-u)
        u = new_u
    c = u*points
    A = la.inv(points.T*np.diag(u)*points - c.T*c)/d
    return np.asarray(A), np.squeeze(np.asarray(c))


def dist_2_cent(x, y, center):
    '''
    Obtain distance to center coordinates for the entire x,y array passed.
    '''

    # delta_x, delta_y = abs(x - center[0]), abs(y - center[1])
    delta_x, delta_y = (x - center[0]), (y - center[1])
    dist = np.sqrt(delta_x ** 2 + delta_y ** 2)

    return delta_x, delta_y, dist


def get_outer_shell(center, x, y):
    '''
    Selects those stars located in an 'outer shell' of the points cloud,
    according to a given accuracy (ie: the 'delta_angle' of the slices the
    circle is divided in).
    '''

    delta_x, delta_y, dist = dist_2_cent(x, y, center)

    # Obtain correct angle with positive x axis for each point.
    angles = []
    for dx, dy in zip(*[delta_x, delta_y]):
        ang = np.rad2deg(np.arctan(abs(dx / dy)))
        if dx > 0. and dy > 0.:
            angles.append(ang)
        elif dx < 0. and dy > 0.:
            angles.append(180. - ang)
        elif dx < 0. and dy < 0.:
            angles.append(270. - ang)
        elif dx > 0. and dy < 0.:
            angles.append(360. - ang)

    # Get indexes of angles from min to max value.
    min_max_ind = np.argsort(angles)

    # Determine sliced circumference. 'delta_angle' sets the number of slices.
    delta_angle = 1.
    circle_slices = np.arange(delta_angle, 361., delta_angle)

    # Fill outer shell with as many empty lists as slices.
    outer_shell = [[] for _ in range(len(circle_slices))]
    # Initialize first angle value (0\degrees) and index of stars in list
    # ordered from min to max distance value to center.
    ang_slice_prev, j = 0., 0
    # For each slice.
    for k, ang_slice in enumerate(circle_slices):
        # Initialize previous maximum distance and counter of stars that have
        # been processed 'p'.
        dist_old, p = 0., 0
        # For each star in the list, except those already processed (ie: with
        # an angle smaller than 'ang_slice_prev')
        for i in min_max_ind[j:]:
            # If the angle is within the slice.
            if ang_slice_prev <= angles[i] < ang_slice:
                # Increase the index that stores the number of stars processed.
                p += 1
                # If the distance to the center is greater than the previous
                # one found (if any).
                if dist[i] > dist_old:
                    # Store coordinates of new star farthest away from center
                    # in this slice.
                    outer_shell[k] = [x[i], y[i]]
                    # Re-assign previous max distance value.
                    dist_old = dist[i]
            # If the angle value is greater than the max slice value.
            elif angles[i] >= ang_slice:
                # Increase index of last star processed and break out of
                # stars loop.
                j += p
                break

        # Re-assign minimum slice angle value.
        ang_slice_prev = ang_slice

    # Remove empty lists from array (ie: slices with no stars in it).
    outer_shell = np.asarray([x for x in outer_shell if x != []])

    return outer_shell


def random_points():
    mu, sigma = np.random.uniform(-10, 10), np.random.uniform(0., 10)
    return mu, sigma


def main():

    # some random points
    N = 20
    mux, sigmax = random_points()
    muy, sigmay = random_points()
    x = np.random.normal(mux, sigmax, N)
    y = np.random.normal(muy, sigmay, N)
    # points0 = np.array(zip(x, y))

    center = [mux, muy]
    points = get_outer_shell(center, x, y)

    print(points)

    # Singular matrix error!
    # points = np.eye(3)

    # A : (d x d) matrix of the ellipse equation in the 'center form':
    # (x-c)' * A * (x-c) = 1
    # 'centroid' is the center coordinates of the ellipse.
    A, centroid = mvee(points)
    # print A

    # V is the rotation matrix that gives the orientation of the ellipsoid.
    # https://en.wikipedia.org/wiki/Rotation_matrix
    # http://mathworld.wolfram.com/RotationMatrix.html
    U, D, V = la.svd(A)

    # x, y radii.
    rx, ry = 1./np.sqrt(D)
    # Major and minor semi-axis of the ellipse.
    dx, dy = 2 * rx, 2 * ry
    a, b = max(dx, dy), min(dx, dy)
    # Eccentricity
    e = np.sqrt(a ** 2 - b ** 2) / a
    # print e

    # print '\n', U
    # print D
    # print V, '\n'
    arcsin = -1. * np.rad2deg(np.arcsin(V[0][0]))
    arccos = np.rad2deg(np.arccos(V[0][1]))
    # Orientation angle (with respect to the x axis counterclockwise).
    alpha = arccos if arcsin > 0. else -1. * arccos
    # print -1*np.rad2deg(np.arcsin(V[0][0])), np.rad2deg(np.arccos(V[0][1]))
    # print np.rad2deg(np.arccos(V[1][0])), np.rad2deg(np.arcsin(V[1][1]))

    # Plot.
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')

    # u, v = np.mgrid[0:2*np.pi:20j, -np.pi/2:np.pi/2:10j]
    # x0 = rx * np.cos(u) * np.cos(v)
    # y0 = ry * np.sin(u) * np.cos(v)
    # E = np.dstack([x0, y0])
    # E = np.dot(E, V) + centroid
    # x1, y1 = np.rollaxis(E, axis=-1)
    # ax.plot(x1, y1)

    # Plot ellipsoid.
    ax = plt.gca()
    ellipse2 = Ellipse(xy=centroid, width=a, height=b, edgecolor='k',
        angle=alpha, fc='None', lw=2)
    ax.add_patch(ellipse2)

    # Plot points.
    plt.scatter(x, y, s=10, zorder=4)
    plt.scatter(points[:, 0], points[:, 1], s=75, c='r', zorder=3)
    # Plot center.
    plt.scatter(*centroid, s=70, c='g')

    plt.show()


if __name__ == "__main__":
    main()