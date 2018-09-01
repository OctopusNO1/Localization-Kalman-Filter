from math import tan, cos, sin
import numpy as np
from numpy import array
import matplotlib.pyplot as plt

from tools import Compute
from tools import Data


class RobotEKF:
    def __init__(self, init_x, wheelbase):
        self.x = init_x
        self.wheelbase = wheelbase

    # predict使用的motion model
    def move(self, x, u, dt):
        latitude = x[1, 0]
        hdg = x[2, 0]   # yaw, rad
        vel = u[0]  # velocity km/h
        steering_angle = u[1]   # 前轮angle rad

        dist = vel * dt     # km

        if abs(steering_angle) > 0.000001:  # is robot turning?
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


yaw_init = 3.00
wheelbase = 0.003  # km
data_length = 130000
second_series, velocity_series, wheel_steering_angle_series, longitude_series, latitude_series\
    = Data.load_data(data_length)
init_x = array([[longitude_series[0]], [latitude_series[0]], [yaw_init]])
ekf_ = RobotEKF(init_x, wheelbase)

longitude_pre_list = [longitude_series[0]]
latitude_pre_list = [latitude_series[0]]
yaw_pre_list = [yaw_init]

for i in range(data_length - 1):  # predict less first
    dt = second_series[i + 1] - second_series[i]

    ekf_.x = ekf_.move(ekf_.x, [velocity_series[i], wheel_steering_angle_series[i]], dt)

    # add
    longitude_pre_list.append(ekf_.x[0, 0])
    latitude_pre_list.append(ekf_.x[1, 0])
    yaw_pre_list.append(ekf_.x[2, 0])
print('end')

print('longitude rmse: ', Compute.root_mean_square_error(longitude_series, longitude_pre_list))
print('latitude rmse', Compute.root_mean_square_error(latitude_series, latitude_pre_list))

plt.scatter(longitude_series, latitude_series, label='measure')
plt.scatter(longitude_pre_list, latitude_pre_list, label='predict')
plt.legend()
plt.show()

