
from filterpy.kalman import ExtendedKalmanFilter
import sympy
from sympy import symbols, Matrix
from numpy import dot

from measure import IMU
from measure.GPS import *
from measure.Lidar import *


class EKF(ExtendedKalmanFilter):
    def __init__(self, wheelbase, std_vel, std_steer):
        ExtendedKalmanFilter.__init__(self, 3, 2, 2)
        self.wheelbase = wheelbase
        self.std_vel = std_vel  # standard deviation of velocity
        self.std_steer = std_steer  # standard deviation of steering

        # 定义符号
        a, x, y, v, w, theta, time = symbols(
            'a, x, y, v, w, theta, t')
        d = v * time    # km
        beta = (d / w) * sympy.tan(a)   # rad
        r = w / sympy.tan(a)    # km

        # FV影响P
        # r_lo = r / 111 / sympy.cos(y / 180 * np.pi)     # km-->longitude
        # r_la = r / 111      # km-->latitude
        # # 定义predict转移函数h形式，单位
        # self.fxu = Matrix(
        #     [[x - r_lo * sympy.sin(theta) + r_lo * sympy.sin(theta + beta)],
        #      [y + r_la * sympy.cos(theta) - r_la * sympy.cos(theta + beta)],
        #      [theta + beta]])

        # 定义predict转移函数h形式，单位
        self.fxu = Matrix(
            [[x - r * sympy.sin(theta) + r * sympy.sin(theta + beta)],
             [y + r * sympy.cos(theta) - r * sympy.cos(theta + beta)],
             [theta + beta]])

        # 定义转移矩阵H和控制矩阵V（对转移函数h求偏导）
        self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
        self.V_j = self.fxu.jacobian(Matrix([v, a]))

        # 定义变量字典save dictionary and it's variables for later use
        self.subs = {x: 0, y: 0, v: 0, a: 0,
                     time: 1/64, w: wheelbase, theta: 0}
        self.x_x, self.x_y, = x, y
        self.v, self.a, self.theta = v, a, theta

    # 此刻的经纬度航向，此刻的速度前轮转角-->后一刻的经纬度航向
    def predict(self, dt, u=0):
        # 计算x
        self.x = IMU.move(self.x, u, dt)

        # 计算P
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

    def update_gps(self, z):
        """
        GPS update
        :param ekf:
        :param z: GPS measurements(longitude, latitude)
        :return:
        """
        self.update(z, HJacobian=H_gps, Hx=hx_gps)

    def update_lidar(self, z, landmark):
        """
        lidar update
        :param ekf:
        :param z: lidar measurement(rang, bearing)
        :param landmark: landmark position(longitude, latitude)
        :return: none
        """
        self.update(z, HJacobian=H_lidar, Hx=hx_lidar, residual=residual,
                    args=(landmark), hx_args=(landmark))
