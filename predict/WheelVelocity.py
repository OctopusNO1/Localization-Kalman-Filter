
import matplotlib.pyplot as plt
from math import acos, asin, sin, cos, tan

from tools import Compute, Data


''' wheel velocity
    turn angle = (wheel_right - wheel_left) / wheelbase *dt
'''
yaw_init = 3.50     # rad,3.33
wheelbase = 0.0015   # km，轮距

data_length = 20000
second_series, velocity_series, car_steering_velocity_series, longitude_series, latitude_series\
    = Data.load_wheel_data(data_length, wheelbase=wheelbase, data_name='../log/log_gps_H9_BLACK_20180902 230927.txt')

longitude_pre_list = [longitude_series[0]]
latitude_pre_list = [latitude_series[0]]
yaw_pre_list = [yaw_init]

print('start')
for i in range(data_length-1):  # predict less first
    # motion cos sin
    dt = second_series[i + 1] - second_series[i]
    car_steer_angle = car_steering_velocity_series[i] * dt

    longitude_pre = longitude_pre_list[i] + Compute.km_lo(
        (velocity_series[i] * cos(yaw_pre_list[i]) * dt), latitude_series[i])
    latitude_pre = latitude_pre_list[i] + Compute.km_la(
        (velocity_series[i] * sin(yaw_pre_list[i]) * dt))
    yaw_pre = yaw_pre_list[i] + car_steer_angle
    # # bicycle， wheel
    # dt = second_series[i + 1] - second_series[i]
    # distance = velocity_series[i] * dt  # km
    #
    # wheel_angle = car_steering_velocity_series[i] * dt
    # # print(i, 'steering angle', steering_angle_series[i])
    # # bicycle model
    # if abs(wheel_angle) > 0.0000001:  # is turn?    1d=0.017rad
    #     turn_angle = distance / wheelbase * tan(wheel_angle)
    #     turn_radius_km = wheelbase / tan(wheelbase)  # km
    #     turn_radius_lo = Compute.km_lo(turn_radius_km, latitude_series[i])
    #     turn_radius_la = Compute.km_la(turn_radius_km)
    #     # print('turn: ', turn_angle)
    #
    #     longitude_pre = longitude_pre_list[i] - turn_radius_lo * sin(yaw_pre_list[i]) \
    #                     + turn_radius_lo * sin(yaw_pre_list[i] + turn_angle)
    #     latitude_pre = latitude_pre_list[i] + turn_radius_la * cos(yaw_pre_list[i]) \
    #                    - turn_radius_la * cos(yaw_pre_list[i] + turn_angle)
    #     yaw_pre = yaw_pre_list[i] + turn_angle
    #     # print('yaw', i+1, ': ', yaw_pre)
    # else:  # no turn
    #     longitude_pre = longitude_pre_list[i] + Compute.km_lo(
    #         (distance * cos(yaw_pre_list[i])), latitude_series[i])
    #     latitude_pre = latitude_pre_list[i] + Compute.km_la(
    #         (distance * sin(yaw_pre_list[i])))
    #     yaw_pre = yaw_pre_list[i]

    # # bicycle, turn
    # dt = second_series[i + 1] - second_series[i]
    # distance = velocity_series[i] * dt  # km
    #
    # turn_angle = car_steering_velocity_series[i] * dt
    # # print(i, 'steering angle', steering_angle_series[i])
    # # bicycle model
    # if abs(turn_angle) > 0.00000000000000001:  # is turn?    1d=0.017rad
    #     turn_radius_km = distance / turn_angle  # km
    #     turn_radius_lo = Compute.km_lo(turn_radius_km, latitude_series[i])
    #     turn_radius_la = Compute.km_la(turn_radius_km)
    #     # print('turn: ', turn_angle)
    #
    #     longitude_pre = longitude_pre_list[i] - turn_radius_lo * sin(yaw_pre_list[i]) \
    #                     + turn_radius_lo * sin(yaw_pre_list[i] + turn_angle)
    #     latitude_pre = latitude_pre_list[i] + turn_radius_la * cos(yaw_pre_list[i]) \
    #                    - turn_radius_la * cos(yaw_pre_list[i] + turn_angle)
    #     yaw_pre = yaw_pre_list[i] + turn_angle
    #     # print('yaw', i+1, ': ', yaw_pre)
    # else:  # no turn
    #     longitude_pre = longitude_pre_list[i] + Compute.km_lo(
    #         (distance * cos(yaw_pre_list[i])), latitude_series[i])
    #     latitude_pre = latitude_pre_list[i] + Compute.km_la(
    #         (distance * sin(yaw_pre_list[i])))
    #     yaw_pre = yaw_pre_list[i]

    longitude_pre_list.append(longitude_pre)
    latitude_pre_list.append(latitude_pre)
    yaw_pre_list.append(yaw_pre)
print('end')

print('longitude rmse: ', Compute.root_mean_square_error(longitude_series, longitude_pre_list))
print('latitude rmse', Compute.root_mean_square_error(latitude_series, latitude_pre_list))

plt.scatter(longitude_series, latitude_series, label='measure')
plt.scatter(longitude_pre_list, latitude_pre_list, label='predict')
plt.legend()
plt.show()



