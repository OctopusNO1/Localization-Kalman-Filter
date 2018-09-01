import sympy
from sympy import symbols, Matrix
from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import dot, array, sqrt
from filterpy.stats import plot_covariance_ellipse
from math import sqrt, tan, cos, sin, atan2
import matplotlib.pyplot as plt
import numpy as np
from numpy.random import randn

from tools import Compute
from tools import Data


# residual = z - h(x)
# h is nonlinear, don't need to linearize


def H_of(x, landmark_pos):
    """ range measure H jacobian
    compute Jacobian of H matrix where h(x) computes
    the range and bearing to a landmark for state x
    state x时的jacobian H
    :param x: state(longitude,latitude,yaw)
            landmark_pos: (longitude,latitude)
    :return: H matrix 2 rows 3 columns, for *x
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


def Hx(x, landmark_pos):
    """ range measure h (x,y,yaw)-->(range,bearing)
    takes a state variable and returns the measurement
    that would correspond to that state.
    :param x: state(longitude,latitude,yaw)
            landmark_pos: (longitude,latitude)
    :return: x-->measurements(range, bearing)
    """
    px = landmark_pos[0]
    py = landmark_pos[1]

    dist = sqrt((px - x[0, 0])**2 + (py - x[1, 0])**2)
    Hx = array([[dist], [atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
    return Hx


def H_GPS(x):
    """
    GPS measure H
    :param x: state(longitude,latitude,yaw)
    :return:
    """
    return array([[1, 0, 0], [0, 1, 0]])


def Hx_GPS(x):
    """
    state x --> GPS measurements (x,y)
    :param x: state(x,y,yaw)
    :return: z measurements(x,y)
    """
    Hx_GPS = array([[x[0]], [x[1]]])
    return Hx_GPS


class RobotEKF(EKF):
    def __init__(self, dt, wheelbase, std_vel, std_steer):
        EKF.__init__(self, 3, 2, 2)
        self.dt = dt
        self.wheelbase = wheelbase
        self.std_vel = std_vel  # standard deviation of velocity
        self.std_steer = std_steer  # standard deviation of steering

        # 定义符号
        a, x, y, v, w, theta, time = symbols(
            'a, x, y, v, w, theta, t')
        d = v * time    # km
        beta = (d / w) * sympy.tan(a)   # rad
        r = w / sympy.tan(a)    # km
        r_lo = r / 111 / sympy.cos(y / 180 * np.pi)
        r_la = r / 111

        # 定义predict转移函数h形式，单位
        self.fxu = Matrix(
            [[x - r_lo * sympy.sin(theta) + r_lo * sympy.sin(theta + beta)],
             [y + r_la * sympy.cos(theta) - r_la * sympy.cos(theta + beta)],
             [theta + beta]])

        # 定义转移矩阵H和控制矩阵V（对转移函数h求偏导）
        self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
        self.V_j = self.fxu.jacobian(Matrix([v, a]))

        # 定义变量字典save dictionary and it's variables for later use
        self.subs = {x: 0, y: 0, v: 0, a: 0,
                     time: dt, w: wheelbase, theta: 0}
        self.x_x, self.x_y, = x, y
        self.v, self.a, self.theta = v, a, theta

    # 此刻的经纬度航向，此刻的速度前轮转角-->后一刻的经纬度航向
    def predict(self, u=0):
        # 调用motion model
        self.x = self.move(self.x, u, self.dt)

        # 代入yaw,velocity,前轮转角
        self.subs[self.theta] = self.x[2, 0]
        self.subs[self.v] = u[0]
        self.subs[self.a] = u[1]

        # 代入FVM
        F = array(self.F_j.evalf(subs=self.subs)).astype(float)
        V = array(self.V_j.evalf(subs=self.subs)).astype(float)

        # covariance of motion noise in control space
        M = array([[self.std_vel * u[0] ** 2, 0],
                   [0, self.std_steer ** 2]])

        # math-->code
        self.P = dot(F, self.P).dot(F.T) + dot(V, M).dot(V.T)

    # predict使用的motion model
    def move(self, x, u, dt):
        latitude = x[1, 0]
        hdg = x[2, 0]   # yaw, rad
        vel = u[0]  # velocity km/h
        steering_angle = u[1]   # 前轮angle rad
        dist = vel * dt     # km

        if abs(steering_angle) > 0.001:  # is robot turning?
            beta = (dist / self.wheelbase) * tan(steering_angle)    # turn angle, rad
            r = self.wheelbase / tan(steering_angle)  # turn radius, km
            r_lo = Compute.km_lo(r, latitude)
            r_la = Compute.km_la(r)

            dx = np.array([[-r_lo * sin(hdg) + r_lo * sin(hdg + beta)],
                           [r_la * cos(hdg) - r_la * cos(hdg + beta)],
                           [beta]])
        else:  # moving in straight line
            dx = np.array([[Compute.km_lo(dist * cos(hdg), latitude)],
                           [Compute.km_la(dist * sin(hdg))],
                           [0]])
        return x + dx


def residual(a, b):
    """ range bearing residual normalize
    compute residual (a-b) between measurements containing
    [range, bearing]. Bearing is normalized to [-pi, pi)
    :param a: measurement[1]
            b: h(state)[1]
    :return: normalized residual
    """
    y = a - b   # residual z = measurement - h(state x)
    y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[1] > np.pi:             # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y


def z_landmark(lmark, sim_pos, std_rng, std_brg):
    """
    range residual
    :param lmark: landmark's position(longitude, latitude)
    :param sim_pos: true state(longitude, latitude, yaw)
    :param std_rng: 标准差
    :param std_brg: 标准差
    :return: residual
    """
    x, y = sim_pos[0, 0], sim_pos[1, 0]
    d = np.sqrt((lmark[0] - x) ** 2 + (lmark[1] - y) ** 2)
    a = atan2(lmark[1] - y, lmark[0] - x) - sim_pos[2, 0]
    z = np.array([[d + randn() * std_rng],
                  [a + randn() * std_brg]])
    return z


def ekf_update(ekf, z, landmark):
    """
    lidar range update
    :param ekf:
    :param z: lidar measurement(rang, bearing)
    :param landmark: landmark position(longitude, latitude)
    :return: none
    """
    ekf.update(z, HJacobian=H_of, Hx=Hx,
               residual=residual,
               args=(landmark), hx_args=(landmark))


def ekf_update_gps(ekf, z):
    """
    GPS measurements
    :param ekf:
    :param z: GPS measurements(longitude, latitude)
    :return:
    """
    ekf.update(z, HJacobian=H_GPS, Hx=Hx_GPS)


landmarks = array([[5, 10], [10, 5], [15, 15]])


def run_localization_landmarks(landmarks, std_vel, std_steer,
                     std_range, std_bearing,
                     step=10, ellipse_step=20, ylim=None):
    ekf = RobotEKF(dt, wheelbase=0.5, std_vel=std_vel,
                   std_steer=std_steer)
    ekf.x = array([[2, 6, .3]]).T  # x, y, steer angle
    ekf.P = np.diag([.1, .1, .1])
    ekf.R = np.diag([std_range ** 2, std_bearing ** 2])

    sim_pos = ekf.x.copy()  # simulated position
    # steering command (vel, steering angle radians)
    u = array([1.1, .01])

    plt.figure()
    plt.scatter(landmarks[:, 0], landmarks[:, 1],
                marker='s', s=60)

    track = []
    for i in range(200):
        sim_pos = ekf.move(sim_pos, u, dt / 10.)  # simulate robot
        track.append(sim_pos)

        if i % step == 0:
            ekf.predict(u=u)

            if i % ellipse_step == 0:
                plot_covariance_ellipse(
                    (ekf.x[0, 0], ekf.x[1, 0]), ekf.P[0:2, 0:2],
                    std=6, facecolor='k', alpha=0.3)

            x, y = sim_pos[0, 0], sim_pos[1, 0]
            for lmark in landmarks:
                z = z_landmark(lmark, sim_pos,
                               std_range, std_bearing)
                ekf_update(ekf, z, lmark)

            if i % ellipse_step == 0:
                plot_covariance_ellipse(
                    (ekf.x[0, 0], ekf.x[1, 0]), ekf.P[0:2, 0:2],
                    std=6, facecolor='g', alpha=0.8)
    track = np.array(track)
    plt.plot(track[:, 0], track[:, 1], color='k', lw=2)
    plt.axis('equal')
    plt.title("EKF Robot localization")
    if ylim is not None: plt.ylim(*ylim)
    plt.show()
    return ekf


def run_localization_gps(std_vel, std_steer, std_lo, std_la, u, z, true_, dt,
                         step=10, ellipse_step=20, ylim=None):
    ekf_ = RobotEKF(dt, wheelbase=0.003, std_vel=std_vel, std_steer=std_steer)

    ekf_.x = array([[117.301701, 39.116025, 3.0]]).T  # x, y, yaw
    ekf_.P = np.diag([.1, .1, .1])

    ekf_.R = np.diag([std_lo ** 2, std_la ** 2])

    plt.figure()
    for i in range(data_length-1):
        # update steps
        if i % step == 0:
            ekf_.predict(u=u[i])
            # 画出predict的协方差椭圆
            if i % ellipse_step == 0:
                plot_covariance_ellipse(
                    (ekf_.x[0, 0], ekf_.x[1, 0]), ekf_.P[0:2, 0:2],
                    std=6, facecolor='k', alpha=0.3)

            ekf_update_gps(ekf_, z[i])
            # 画出update后的协方差椭圆
            if i % ellipse_step == 0:
                plot_covariance_ellipse(
                    (ekf_.x[0, 0], ekf_.x[1, 0]), ekf_.P[0:2, 0:2],
                    std=6, facecolor='g', alpha=0.8)
    plt.plot(true_[:, 0], true_[:, 1], color='k', lw=2)
    plt.axis('equal')
    plt.title("EKF Robot localization")
    if ylim is not None: plt.ylim(*ylim)
    plt.show()
    return ekf_


yaw_init = 3.00
wheelbase = 0.003  # km

data_length = 130000
_, _, _, true_longitude_series, true_latitude_series = Data.load_data(data_length)
second_series, velocity_series, wheel_steering_angle_series, longitude_series, latitude_series\
    = Data.load_noise_data(data_length)

longitude_pre_list = [longitude_series[0]]
latitude_pre_list = [latitude_series[0]]
yaw_pre_list = [yaw_init]

u = list(zip(velocity_series, wheel_steering_angle_series))
z = list(zip(longitude_series, latitude_series))
true_ = list(zip(true_longitude_series, true_latitude_series))
dt = 0.02

ekf_ = run_localization_gps(std_vel=0.1, std_steer=np.radians(1), std_lo=0.3, std_la=0.1,
                            u=u, z=z, true_=true_, dt=dt)
print('Final P:', ekf_.P.diagonal())

