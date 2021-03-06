
from math import sin, cos, tan
import matplotlib.pyplot as plt

from tools import Compute, Data


''' bicycle model
    steering wheel angle-->front tire angle-->turn angle
'''
yaw_init = 3.33     # 3.33, 16.3
wheelbase = 0.003  # km

data_length = 20000
second_series, velocity_series, wheel_angle_series, longitude_series, latitude_series\
    = Data.load_data(data_length, correction=0, conversion=16.3, data_name='../log/log_gps_H9_BLACK_20180902 230927.txt')
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
        turn_radius_lo = Compute.km_lo(turn_radius_km, latitude_series[i])
        turn_radius_la = Compute.km_la(turn_radius_km)
        # print('turn: ', turn_angle)

        longitude_pre = longitude_pre_list[i] - turn_radius_lo * sin(yaw_pre_list[i]) \
                        + turn_radius_lo * sin(yaw_pre_list[i] + turn_angle)
        latitude_pre = latitude_pre_list[i] + turn_radius_la * cos(yaw_pre_list[i])\
                       - turn_radius_la * cos(yaw_pre_list[i] + turn_angle)
        yaw_pre = yaw_pre_list[i] + turn_angle
        # print('yaw', i+1, ': ', yaw_pre)
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

