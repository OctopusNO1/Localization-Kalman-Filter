from math import sqrt, atan2
import numpy as np
from numpy import array
from numpy.random import randn


# residual = z - h(x), h is nonlinear, don't need to linearize
def hx_lidar(x, landmark_pos):
    """ range measure h (x,y,yaw)-->(range,bearing)
    takes a state variable and returns the measurement
    that would correspond to that state.
    :param x: state(longitude,latitude,yaw)
            landmark_pos: (longitude,latitude)
    :return: h(x) lidar measurements(range, bearing)
    """
    px = landmark_pos[0]
    py = landmark_pos[1]

    dist = sqrt((px - x[0, 0])**2 + (py - x[1, 0])**2)
    hx_lidar = array([[dist], [atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
    return hx_lidar


def H_lidar(x, landmark_pos):
    """ range measure H jacobian
    compute Jacobian of H matrix where h(x) computes
    the range and bearing to a landmark for state x
    state x时的jacobian H
    :param x: state(longitude,latitude,yaw)
            landmark_pos: (longitude,latitude)
    :return: H jacobian matrix(2 rows 3 columns), for *x
    """
    # 地标的经纬度
    px = landmark_pos[0]
    py = landmark_pos[1]

    # 单位：测量的px,py是地标的经度纬度，state x是（经度，纬度，航向角rad）
    hyp = (px - x[0, 0])**2 + (py - x[1, 0])**2
    dist = sqrt(hyp)
    H = array(
        [[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
         [ (py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
    return H


def z_landmark(lmark, sim_pos, std_rng, std_brg):
    """
    simulate lidar measurements
    :param lmark: landmark's position(longitude, latitude)
    :param sim_pos: true state(longitude, latitude, yaw)
    :param std_rng: 标准差，模拟测量的随机性
    :param std_brg: 标准差，模拟测量的随机性
    :return: z lidar measurements
    """
    x, y = sim_pos[0, 0], sim_pos[1, 0]
    d = np.sqrt((lmark[0] - x) ** 2 + (lmark[1] - y) ** 2)
    a = atan2(lmark[1] - y, lmark[0] - x) - sim_pos[2, 0]
    z = np.array([[d + randn() * std_rng],
                  [a + randn() * std_brg]])
    return z


def residual(a, b):
    """ normalize lidar bearing residual
    compute residual (a-b) between measurements containing
    [range, bearing]. Bearing is normalized to [-pi, pi)
    :param a: measurement[1] bearing
            b: h(state)[1] bearing
    :return: normalized bearing residual
    """
    y = a - b  # residual z = measurement - h(state x)
    y[1] = y[1] % (2 * np.pi)  # force in range [0, 2 pi)
    if y[1] > np.pi:  # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y


# landmarks = array([[5, 10], [10, 5], [15, 15]])
# def run_localization_landmarks(landmarks, std_vel, std_steer,
#                      std_range, std_bearing,
#                      step=10, ellipse_step=20, ylim=None):
#     ekf = RobotEKF(dt, wheelbase=0.5, std_vel=std_vel,
#                    std_steer=std_steer)
#     ekf.x = array([[2, 6, .3]]).T  # x, y, steer angle
#     ekf.P = np.diag([.1, .1, .1])
#     ekf.R = np.diag([std_range ** 2, std_bearing ** 2])
#
#     sim_pos = ekf.x.copy()  # simulated position
#     # steering command (vel, steering angle radians)
#     u = array([1.1, .01])
#
#     plt.figure()
#     plt.scatter(landmarks[:, 0], landmarks[:, 1],
#                 marker='s', s=60)
#
#     track = []
#     for i in range(200):
#         sim_pos = ekf.move(sim_pos, u, dt / 10.)  # simulate robot
#         track.append(sim_pos)
#
#         if i % step == 0:
#             ekf.predict(u=u)
#
#             if i % ellipse_step == 0:
#                 plot_covariance_ellipse(
#                     (ekf.x[0, 0], ekf.x[1, 0]), ekf.P[0:2, 0:2],
#                     std=6, facecolor='k', alpha=0.3)
#
#             x, y = sim_pos[0, 0], sim_pos[1, 0]
#             for lmark in landmarks:
#                 z = z_landmark(lmark, sim_pos,
#                                std_range, std_bearing)
#                 ekf_update(ekf, z, lmark)
#
#             if i % ellipse_step == 0:
#                 plot_covariance_ellipse(
#                     (ekf.x[0, 0], ekf.x[1, 0]), ekf.P[0:2, 0:2],
#                     std=6, facecolor='g', alpha=0.8)
#     track = np.array(track)
#     plt.plot(track[:, 0], track[:, 1], color='k', lw=2)
#     plt.axis('equal')
#     plt.title("EKF Robot localization")
#     if ylim is not None: plt.ylim(*ylim)
#     plt.show()
#     return ekf
#
