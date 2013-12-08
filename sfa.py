import numpy as np
import matplotlib.pyplot as plt
from geometry import *
from math import *
from random import randrange
import mdp

def random_walk(length, sensor_positions, N, x_max, y_max, walls):
    # initialize vectors of x and y positions in the room
    positions = np.zeros((2, length))
    # start in the middle of the room
    positions[0, 0], positions[1, 0] = x_max/2, y_max/2
    # the length between the change of speed and direction
    n = 25
    # some initial random direction, x and y coordinates
    rand_dir = np.array(2 * np.random.rand(2) - 1)
    distances = np.zeros((trajectory, N))
    # speed of the bat
    speed = 2
    # iterate through cycles of length n
    for i in range(0, length, n):
        # the angle of rotation in radians, divided by the length n (small)
        angle_rad = radians(randrange(0, 40))/n
        # rotation matrix
        rotation_mat = np.array([[cos(angle_rad), -sin(angle_rad)], [sin(angle_rad), cos(angle_rad)]])
        for j in range(i, i+n):
            # start from the second position, the first one is already determined
            if j==0:
                distances[0, :] = sensory_input((positions[0, j], positions[1, j]), sensor_positions, N, x_max, y_max, walls)
                continue
            # calculate the new random direction rotated by the angle
            rand_dir = np.dot(rotation_mat, rand_dir)
            # move the bat to the next position
            positions[:, j] = positions[:, j-1] + rand_dir*speed
            # take care of the walls and change the rand_dir appropriately
            if positions[0, j] > x_max or positions[0, j]<0:
                rand_dir[0] = -rand_dir[0]
            if positions[1, j] > y_max or positions[1, j]<0:
                rand_dir[1] = -rand_dir[1]
            positions[:, j] = positions[:, j-1] + rand_dir*speed
            # calculate the distances to the walls for each sensor from this position
            distances[j, :] = sensory_input((positions[0, j], positions[1, j]), sensor_positions, N, x_max, y_max, walls)
    return positions, distances

def get_sensor_positions(N, orthogonal=False):
    # determine what the sensors are facing, i.e. the angle of their orientation
    # the special case where the rays have to be orthogonal
    if N == 2 and orthogonal:
        sensor_positions = [0, 90]
    else:
        # random angles at which the rays are
        sensor_positions = [randrange(0, 360) for i in range(N)]
    return sensor_positions # in degrees

def sensory_input(curr_position, sensor_positions, N, x_max, y_max, walls):
    # the 4 walls of the room
    distances = []
    for i in sensor_positions:
        # handle some special angles as 0, 90, 180 and 270
        if i == 0:
            intersection = (x_max, curr_position[1])
            distances.append(np.linalg.norm(np.array(curr_position)-np.array(intersection)))
        elif i == 90:
            intersection = (curr_position[0], y_max)
            distances.append(np.linalg.norm(np.array(curr_position)-np.array(intersection)))
        elif i == 180:
            intersection = (0, curr_position[1])
            distances.append(np.linalg.norm(np.array(curr_position)-np.array(intersection)))
        elif i == 270:
            intersection = (curr_position[0], 0)
            distances.append(np.linalg.norm(np.array(curr_position)-np.array(intersection)))
        else:
            # calculate the shifts as cos(angle) and sin(angle)
            x_shift = cos(radians(i))
            y_shift = sin(radians(i))
            # end position of the ray
            end_position = (curr_position[0]+(x_shift*100*x_max), curr_position[1]+(y_shift*100*y_max))
            intersection = None
            # check with which wall the ray intersects
            for j in walls:
                intersection = calculateIntersectPoint(curr_position, end_position, j[0], j[1])
                if intersection:
                    # calculate the distance from the wall
                    distances.append(np.linalg.norm(np.array(curr_position)-np.array(intersection)))
                    break
    return np.array(distances)

def sfa(N, sensor_positions, x_max, y_max, walls, pol_degree=1, whitening=False, ica=False, num_outputs=None):
    # polynomial expansion node of certain degree
    flow = mdp.nodes.PolynomialExpansionNode(pol_degree)
    # add the whitening node
    if whitening:
        flow += mdp.nodes.WhiteningNode(svd=True, reduce=True)
    # SFA node
    flow += mdp.nodes.SFANode(output_dim=num_outputs)
    # add the ICA node
    if ica:
        flow += mdp.nodes.CuBICANode()
    # calculate the data collected by the sensors at each position in the room
    sensor_data = np.zeros((y_max, x_max, N)) # y and x as on a plot
    for x in range(0, x_max):
        for y in range(0, y_max):
            sensor_data[y, x, :] = sensory_input((x, y), sensor_positions, N, x_max, y_max, walls)
    # perform the random walk
    positions, distances = random_walk(trajectory, sensor_positions, N, x_max, y_max, walls)
    # plot_rand_walk(positions, x_max, y_max) # for debugging
    # train the nodes
    flow.train(distances)
    grid = np.reshape(sensor_data, (y_max*x_max, N))
    # extract the slowly varying features
    slow_features = flow(grid)
    return positions, slow_features
    

def plot_rand_walk(positions, x_max, y_max):
    plt.figure()
    plt.plot(positions[0], positions[1])
    plt.xlim([0, x_max])
    plt.ylim([0, y_max])
    plt.title('Random walk of a bat in a room of dimensions %ix%i'%(x_max, y_max))
    plt.axis('equal')

def plot_SFs(slow_features, x_max, y_max, num_plots=None):
    plt.figure()
    plt.suptitle("Slow features")
    if (not num_plots) or (num_plots > slow_features.shape[1]):
        num_plots = slow_features.shape[1]
    for i in range(num_plots):
        plt.subplot(ceil(float(num_plots)/3), 3, i+1)
        plt.imshow(np.reshape(slow_features[:,i], (y_max, x_max)))
        plt.title("SF " + str(i+1))
        plt.axis('off')

def plot_sensor_data(sensor_data, N):
    plt.figure()
    plt.suptitle("Sensor data")
    for i in range(N):
        plt.subplot(3, ceil(float(N)/3), i+1)
        plt.imshow(sensor_data[:,:,i])
        plt.title("sensor " + str(i+1))


if __name__=='__main__':
    # room dimensions
    x_max = 200 # width
    y_max = 100 # height
    # walls defined by 2 points each
    walls = [((0, 0), (0, y_max)), ((0, 0), (x_max, 0)), ((x_max, 0), (x_max, y_max)), ((0, y_max), (x_max, y_max))]
    # number of steps for the random walk
    trajectory = 10000
    # number of sensors
    N = 2
    # determine positions of the sensors (what they are facing = angles)
    sensor_positions = get_sensor_positions(N, orthogonal=True)
    # do SFA
    positions, slow_features = sfa(N, sensor_positions, x_max, y_max, walls)
    # plot the random walk - 1.
    plot_rand_walk(positions, x_max, y_max)
    # plot the slow features - 3.
    plot_SFs(slow_features, x_max, y_max)

    # do the same for a quadratic room
    x_max = 300 # width
    y_max = 300 # height
    # walls defined by 2 points each
    walls = [((0, 0), (0, y_max)), ((0, 0), (x_max, 0)), ((x_max, 0), (x_max, y_max)), ((0, y_max), (x_max, y_max))]
    # number of steps for the random walk
    trajectory = 25000
    # do SFA
    positions, slow_features = sfa(N, sensor_positions, x_max, y_max, walls)
    plot_SFs(slow_features, x_max, y_max)

    # more, random sensors - 4.
    # number of sensors
    N = 5
    # determine positions of the sensors (what they are facing = angles)
    sensor_positions = get_sensor_positions(N)

    # vary polynomial degree, squared room - 5.
    for poldegree in [1, 3, 5, 7]:
        # do SFA
        positions, slow_features = sfa(N, sensor_positions, x_max, y_max, walls, pol_degree=poldegree, whitening=True)
        plot_SFs(slow_features, x_max, y_max, num_plots=21)

    # vary polynomial degree, elongated room - 5.
    # room dimensions
    x_max = 1000 # width
    y_max = 300 # height
    # walls defined by 2 points each
    walls = [((0, 0), (0, y_max)), ((0, 0), (x_max, 0)), ((x_max, 0), (x_max, y_max)), ((0, y_max), (x_max, y_max))]
    # number of steps for the random walk - need more for bigger rooms
    trajectory = 50000
    for poldegree in [1, 3, 5, 7]:
        # do SFA
        positions, slow_features = sfa(N, sensor_positions, x_max, y_max, walls, pol_degree=poldegree, whitening=True)
        plot_SFs(slow_features, x_max, y_max, num_plots=21)

    # # use ICA - 6.
    positions, slow_features = sfa(N, sensor_positions, x_max, y_max, walls, pol_degree=5, whitening=True, ica=True)
    plot_SFs(slow_features, x_max, y_max, num_plots=21)
    # with less features
    positions, slow_features = sfa(N, sensor_positions, x_max, y_max, walls, pol_degree=5, whitening=True, ica=True, num_outputs=N*2)
    plot_SFs(slow_features, x_max, y_max)
    plt.show()
