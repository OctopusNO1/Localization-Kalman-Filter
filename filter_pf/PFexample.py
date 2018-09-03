
import matplotlib.pyplot as plt
import scipy
import scipy.stats
import numpy as np
# from numpy import random
from numpy.random import uniform, randn
from numpy.linalg import norm
from numpy.random import seed
from filterpy.monte_carlo import systematic_resample


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


def predict(particles, u, std, dt=1.):
    """
    move participles不需要准确的motion model
    according to control input u (heading change, velocity)
    with noise Q (std heading change, std velocity)`
    :param particles:
    :param u:
    :param std: 标准差，Q
    :param dt:
    :return:
    """
    N = len(particles)
    # update heading
    particles[:, 2] += u[0] + (randn(N) * std[0])
    particles[:, 2] %= 2 * np.pi

    # move in the (noisy) commanded direction
    dist = (u[1] * dt) + (randn(N) * std[1])
    particles[:, 0] += np.cos(particles[:, 2]) * dist
    particles[:, 1] += np.sin(particles[:, 2]) * dist


def update(particles, weights, z, R, landmarks):
    """
    根据distance更新weights
    :param particles: states(x,y,yaw)*N
    :param weights:
    :param z: measurements-robot和landmarks的distance
    :param R: measure covariance
    :param landmarks: (x,y)
    :return:
    """
    weights.fill(1.)
    for i, landmark in enumerate(landmarks):
        distance = np.linalg.norm(particles[:, 0:2] - landmark, axis=1)     # particles和landmarks的distance
        weights *= scipy.stats.norm(distance, R).pdf(z[i])  #

    weights += 1.e-300      # avoid round-off to zero
    weights /= sum(weights)     # normalize


def estimate(particles, weights):
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
    :return:
    """
    return 1. / np.sum(np.square(weights))


def resample_from_index(particles, weights, indexes):
    particles[:] = particles[indexes]
    weights[:] = weights[indexes]
    weights.fill (1.0 / len(weights))


def run_pf1(N, iters=18, sensor_std_err=.1,
            plot_particles=False,
            xlim=(0, 20), ylim=(0, 20),
            initial_x=None):
    '''

    :param N: 粒子数量
    :param iters:
    :param sensor_std_err:
    :param plot_particles:
    :param xlim:
    :param ylim:
    :param initial_x:
    :return:
    '''
    landmarks = np.array([[-1, 2], [5, 10], [12, 14], [18, 21]])
    NL = len(landmarks)

    plt.figure()

    # create particles and weights
    if initial_x is not None:   # 有最初的x就创建gaussian，std与Q
        particles = create_gaussian_particles(
            mean=initial_x, std=(5, 5, np.pi / 4), N=N)
    else:
        particles = create_uniform_particles((0, 20), (0, 20), (0, 6.28), N)
    weights = np.zeros(N)

    # 画出透明粒子
    if plot_particles:
        alpha = .20
        if N > 5000:
            alpha *= np.sqrt(5000) / np.sqrt(N)
        plt.scatter(particles[:, 0], particles[:, 1],
                    alpha=alpha, color='g')

    xs = []     # estimate x
    robot_pos = np.array([0., 0.])  # true position
    for x in range(iters):
        robot_pos += (1, 1)     # 模拟true move

        # distance from robot to each landmark
        zs = (norm(landmarks - robot_pos, axis=1) +
              (randn(NL) * sensor_std_err))

        # move diagonally forward to (x+1, x+1)
        predict(particles, u=(0.00, 1.414), std=(.2, .05))

        # incorporate measurements
        update(particles, weights, z=zs, R=sensor_std_err,
               landmarks=landmarks)

        # resample if too few effective particles
        if neff(weights) < N / 2:
            indexes = systematic_resample(weights)
            resample_from_index(particles, weights, indexes)    # 重采样

        mu, var = estimate(particles, weights)
        xs.append(mu)

        if plot_particles:
            plt.scatter(particles[:, 0], particles[:, 1],
                        color='k', marker=',', s=1)
        p1 = plt.scatter(robot_pos[0], robot_pos[1], marker='+',
                         color='k', s=180, lw=3)
        p2 = plt.scatter(mu[0], mu[1], marker='s', color='r')

    # xs = np.array(xs)
    # plt.plot(xs[:, 0], xs[:, 1])
    plt.legend([p1, p2], ['Actual', 'PF'], loc=4, numpoints=1)
    plt.xlim(*xlim)
    plt.ylim(*ylim)
    print('final position error, variance:\n\t', mu - np.array([iters, iters]), var)
    plt.show()


seed(2)
run_pf1(N=5000, plot_particles=False)
#
# seed(2)
# run_pf1(N=5000, iters=8, plot_particles=True,
#         xlim=(0,8), ylim=(0,8))

# seed(2)
# run_pf1(N=100000, iters=8, plot_particles=True,
#         xlim=(0,8), ylim=(0,8))

# seed(6)
# run_pf1(N=5000, plot_particles=True, ylim=(-20, 20))

# seed(6)
# run_pf1(N=5000, plot_particles=True, initial_x=(1,1, np.pi/4))
