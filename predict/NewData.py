
from math import sin, cos, tan
import matplotlib.pyplot as plt
from numpy import radians


''' bicycle model
    steering wheel angle-->front tire angle-->turn angle
'''
import pandas as pd
# from numpy import tan

from tools import Compute


''' load data
'''
data_length = 10000
# yaw_init = radians(-167.718)
# yaw_init = radians(99.568)
yaw_init = radians(-61.219)
wheelbase = 0.0028  # km

# data_frame = pd.read_table('../log/log_gps_H9_BLACK_20180902 230927.txt',
#                            header=0, delim_whitespace=True, nrows=data_length)
# data_frame = pd.read_table('../log/log_gps_H9_BLACK_20180902 235108.txt',
#                            header=0, delim_whitespace=True, nrows=data_length)
data_frame = pd.read_table('../log/log_gps_H9_BLACK_20180903 013352.txt',
                           header=0, delim_whitespace=True, nrows=data_length)
second_series = data_frame.loc[:, '时间']
longitude_series = data_frame.loc[:, '经度']
latitude_series = data_frame.loc[:, '纬度']
velocity_series = Compute.kmh_kms(data_frame.loc[:, '速度'])   # kms
steering_wheel_angle_series = Compute.deg_rad(data_frame.loc[:, '实际转向角'])    # 方向盘转角, rad
wheel_angle_series = -steering_wheel_angle_series/20  # 前轮转角, rad

longitude_pre_list = [longitude_series[0]]
latitude_pre_list = [latitude_series[0]]
yaw_pre_list = [yaw_init]
# print(0, '时刻，经度：', longitude_series[0])
# print(0, '时刻，纬度：', latitude_series[0])
# print(0, '时刻，航向：', yaw_init)

print('start')
for i in range(data_length - 1):  # predict less first
    dt = second_series[i + 1] - second_series[i]
    distance = velocity_series[i] * dt  # km

    # bicycle model
    if abs(wheel_angle_series[i]) > 0.000001:  # is turn?    1d=0.017rad
        # 方向盘转角-->车辆转向角
        turn_angle = distance / wheelbase * tan(wheel_angle_series[i])   # rad, wheelbase?
        turn_radius_km = wheelbase / tan(wheel_angle_series[i])     # km
        # ???
        turn_radius_lo = Compute.km_lo(turn_radius_km, latitude_series[i])
        turn_radius_la = Compute.km_la(turn_radius_km)

        longitude_pre = longitude_pre_list[i] - turn_radius_lo * sin(yaw_pre_list[i]) \
                        + turn_radius_lo * sin(yaw_pre_list[i] + turn_angle)
        latitude_pre = latitude_pre_list[i] + turn_radius_la * cos(yaw_pre_list[i])\
                       - turn_radius_la * cos(yaw_pre_list[i] + turn_angle)
        yaw_pre = yaw_pre_list[i] + turn_angle
    else:  # no turn
        longitude_pre = longitude_pre_list[i] + Compute.km_lo(
            (distance * cos(yaw_pre_list[i])), latitude_series[i])
        latitude_pre = latitude_pre_list[i] + Compute.km_la(
            (distance * sin(yaw_pre_list[i])))
        yaw_pre = yaw_pre_list[i]

    # print(i+1, '时刻，经度：', longitude_pre)
    # print(i+1, '时刻，纬度：', latitude_pre)
    # print(i+1, '时刻，航向：', yaw_init)

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

