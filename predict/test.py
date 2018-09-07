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
    def move(self, x, u, dt=0.02):
        latitude = x[1, 0]
        yaw = x[2, 0]  # rad
        vel = u[0]  # km/s
        wheel_angle = u[1]  # 前轮 angle, rad
        dist = vel * dt  # km

        if abs(wheel_angle) > 0.000001:  # is robot turning?
            turn_angle = (dist / wheelbase) * tan(wheel_angle)  # rad
            turn_radius = wheelbase / tan(wheel_angle)  # km

            r_lo = Compute.km_lo(turn_radius, latitude)
            r_la = Compute.km_la(turn_radius)

            dx = np.array([[-r_lo * sin(yaw) + r_lo * sin(yaw + turn_angle)],
                           [r_la * cos(yaw) - r_la * cos(yaw + turn_angle)],
                           [turn_angle]])
        else:  # moving in straight line
            dx = np.array([[Compute.km_lo(dist * cos(yaw), latitude)],
                           [Compute.km_la(dist * sin(yaw))],
                           [0]])
        return x + dx


yaw_init = 3.00
wheelbase = 0.003  # km
data_length = 15000
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

