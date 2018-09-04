
import matplotlib.pyplot as plt
import scipy
import scipy.stats
import numpy as np
# from numpy import random
from numpy.random import uniform, randn
from numpy.linalg import norm
from numpy.random import seed
from filterpy.monte_carlo import systematic_resample

from tools import Compute
from tools import Data

''' particle——state(x, y, yaw)
    
'''


def create_uniform_particles(x_range, y_range, hdg_range, N):
    """
    创建均匀分布的粒子堆
    :param x_range: (x_min, x_max)
    :param y_range: (y_min. y_max)
    :param hdg_range: 航向(yaw_min, yaw_max)
    :param N: 粒子数量
    :return: array([[x, y, yaw]*N ])
    """
    particles = np.empty((N, 3))
    particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
    particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
    particles[:, 2] = uniform(hdg_range[0], hdg_range[1], size=N)
    particles[:, 2] %= 2 * np.pi
    return particles


def create_gaussian_particles(mean, std, N):
    """
    创建高斯分布的粒子堆
    :param mean:
    :param std:
    :param N:
    :return:
    """
    particles = np.empty((N, 3))
    particles[:, 0] = mean[0] + (randn(N) * std[0])
    particles[:, 1] = mean[1] + (randn(N) * std[1])
    particles[:, 2] = mean[2] + (randn(N) * std[2])
    particles[:, 2] %= 2 * np.pi
    return particles


def predict(particles, u, std, dt=0.02):
    """
    move participles，不需要准确的motion model
    according to control input with noise Q
    :param particles:
    :param u: (heading change, velocity)
    :param std: 标准差，Q(std heading change, std velocity)
    :param dt: default 0.02， 根据data算的更精确
    :return: none
    """
    N = len(particles)
    # update heading车辆转角，不准
    particles[:, 2] += u[0] + (randn(N) * std[0])
    particles[:, 2] %= 2 * np.pi

    # move in the (noisy) commanded direction
    # N个不同的distance
    dist = (u[1] * dt) + (randn(N) * std[1])    # km
    # predict时不知道true latitude
    # 用各个particle的latitude——int/list
    for i in range(len(particles)):
        yaw = np.cos(particles[i, 2])
        dist_lo = Compute.km_lo(dist[i], particles[i, 1])
        particles[i, 0] += yaw * dist_lo   # km-->经度
    particles[:, 1] += np.sin(particles[:, 2]) * dist/111   # km-->纬度


def update(particles, weights, z, R, landmarks):
    """
    根据distance更新weights
    :param particles: states(x,y,yaw)*N
    :param weights:
    :param z: measurements：distance between robot and landmarks
    :param R: measure covariance
    :param landmarks: (x,y)*N
    :return:
    """
    weights.fill(1.)
    for i, landmark in enumerate(landmarks):
        distance = np.linalg.norm(particles[:, 0:2] - landmark, axis=1)     # particles和landmarks的distance
        weights *= scipy.stats.norm(distance, R).pdf(z[i])  #

    weights += 1.e-300      # avoid round-off to zero
    weights /= sum(weights)     # normalize


def estimate(particles, weights):
    """

    :param particles: states(x, y, yaw)*N
    :param weights:
    :return: mean
            variance
    """
    """returns mean and variance of the weighted particles"""
    pos = particles[:, 0:2]
    mean = np.average(pos, weights=weights, axis=0)
    var = np.average((pos - mean)**2, weights=weights, axis=0)
    return mean, var


# def simple_resample(particles, weights):
#     N = len(particles)
#     cumulative_sum = np.cumsum(weights)
#     cumulative_sum[-1] = 1. # avoid round-off error
#     indexes = np.searchsorted(cumulative_sum, random(N))
#
#     # resample according to indexes
#     particles[:] = particles[indexes]
#     weights.fill(1.0 / N)


def neff(weights):
    """
    resample的判断标准
    :param weights:
    :return: threshold
    """
    return 1. / np.sum(np.square(weights))


def resample_from_index(particles, weights, indexes):
    """
    resample
    :param particles:
    :param weights:
    :param indexes: 生成的
    :return:
    """
    particles[:] = particles[indexes]
    weights[:] = weights[indexes]
    weights.fill (1.0 / len(weights))


def run_pf1(N, iters=13, sensor_std_err=0.0000001,
            plot_particles=False, is_lim=False,
            xlim=(117.21, 117.31), ylim=(39.11, 39.21),
            initial_x=None):
    '''

    :param N: 粒子数量
    :param iters: data length
    :param sensor_std_err: measurement/sensor standard测量的准不准？
    :param plot_particles: 是否画出粒子堆
    :param xlim: x刻度
    :param ylim: y刻度
    :param initial_x: 知道初始位置就gaussian，不知道就均匀分布
    :return: none
    '''
    landmarks = np.array([[117.3005, 39.1160], [117.2995, 39.1160], [117.2985, 39.1160], [117.2975, 39.1160],
                          [117.3005, 39.1170], [117.2995, 39.1170], [117.2985, 39.1170], [117.2975, 39.1170],
                          [117.2965, 39.1160], [117.2960, 39.1160], [117.2955, 39.1160], [117.2950, 39.1160],
                          [117.2965, 39.1170], [117.2960, 39.1170], [117.2955, 39.1170], [117.2950, 39.1170]])
    NL = len(landmarks)

    plt.figure()

    # create particles and weights
    if initial_x is not None:   # 有最初的x就创建gaussian，std与Q
        particles = create_gaussian_particles(
            mean=initial_x, std=(0.00001, 0.00001, np.pi / 8), N=N)
    else:
        particles = create_uniform_particles((117.21, 117.31), (39.11, 39.21), (0, np.pi*2), N)
    weights = np.zeros(N)

    # 画出初始化的透明粒子，画出地标
    if plot_particles:
        alpha = .20
        if N > 5000:
            alpha *= np.sqrt(5000) / np.sqrt(N)
        plt.scatter(particles[:, 0], particles[:, 1],
                    alpha=alpha, color='g')
        plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='s', s=60)

    xs = []     # estimate x
    dt_series, turn_angle_series, velocity_series, longitude_series, latitude_series\
        = Data.load_process_data(data_length=iters, correction=1.3, conversion=17)     # true position

    true_pos, us = [], []  # array([[longitude], [latitude]])
    for (turn_angle, velocity, longitude, latitude) in zip(turn_angle_series, velocity_series, longitude_series, latitude_series):
        true_pos.append([longitude, latitude])
        us.append([turn_angle, velocity])
    true_pos = np.array(true_pos)
    us = np.array(us)

    for i in range(iters):
        # distance from robot to each landmark
        zs = (norm(landmarks - true_pos[i], axis=1) +
              (randn(NL) * sensor_std_err))

        # move diagonally forward to (x+1, x+1)
        predict(particles, u=us[i], std=(0.2, 0.05), dt=dt_series[i])   # predict的置信度

        # incorporate measurements
        update(particles, weights, z=zs, R=sensor_std_err,  # measure的置信度
               landmarks=landmarks)

        # resample if too few effective particles
        if neff(weights) < N / 2:
            indexes = systematic_resample(weights)
            resample_from_index(particles, weights, indexes)    # 重采样

        mu, var = estimate(particles, weights)
        xs.append(mu)

        # 画出estimate后的粒子堆
        if plot_particles:
            plt.scatter(particles[:, 0], particles[:, 1],
                        color='k', marker=',', s=1)

        # 画出真实位置点+
        p1 = plt.scatter(true_pos[i, 0], true_pos[i, 1], marker='+',
                         color='k', s=180, lw=1, label='true')
        # 画出estimate位置点
        p2 = plt.scatter(mu[0], mu[1], marker='s', color='r', label='estimate')

    # xs = np.array(xs)
    # plt.plot(xs[:, 0], xs[:, 1])
    plt.legend([p1, p2], ['Actual', 'PF'], loc=4, numpoints=1)
    if is_lim:
        plt.xlim(*xlim)
        plt.ylim(*ylim)
    print('final position error, variance:\n\t', mu - np.array([iters, iters]), var)
    plt.show()


# seed(2)
# run_pf1(N=5000, plot_particles=False)
#
# seed(2)
# run_pf1(N=5000, iters=8, plot_particles=True,
#         xlim=(0, 8), ylim=(0, 8))

# seed(2)
# run_pf1(N=100000, iters=8, plot_particles=True,
#         xlim=(0,8), ylim=(0,8))

# seed(6)
# run_pf1(N=5000, plot_particles=True, ylim=(-20, 20))

seed(6)
run_pf1(N=5000, iters=130, plot_particles=False, initial_x=(117.301701, 39.116025, 3), is_lim=False)
