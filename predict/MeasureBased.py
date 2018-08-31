
import matplotlib.pyplot as plt
from math import acos, asin

from tools import Compute, Data


''' measure based model
    x_ = x + v * cos_yaw * dt
    yaw is based on measurements
'''
data_length = 1300
second_series, velocity_series, _, longitude_series, latitude_series = Data.load_data(data_length)

longitude_pre_list = [longitude_series[0]]
latitude_pre_list = [latitude_series[0]]

print('start')
for i in range(data_length-1):  # predict less first
    dt = second_series[i+1] - second_series[i]
    # dt = 0.02
    # 对GPS测量的依赖过大：1、前一时刻的经纬度    2、方向
    # cos_yaw = Compute.cos_yaw(longitude_series[i], latitude_series[i], longitude_series[i + 1], latitude_series[i + 1])
    # sin_yaw = Compute.sin_yaw(longitude_series[i], latitude_series[i], longitude_series[i + 1], latitude_series[i + 1])
    #
    # longitude_pre = longitude_series[i] + Compute.km_lo(
    #     (velocity_series[i] * cos_yaw * dt), latitude_series[i])
    # latitude_pre = latitude_series[i] + Compute.km_la(
    #     (velocity_series[i] * sin_yaw * dt))

    # 方向依赖于GPS测量值
    cos_yaw = Compute.cos_yaw(longitude_pre_list[i], latitude_pre_list[i], longitude_series[i + 1], latitude_series[i + 1])
    sin_yaw = Compute.sin_yaw(longitude_pre_list[i], latitude_pre_list[i], longitude_series[i + 1], latitude_series[i + 1])
    print('yaw', i, ': ', acos(cos_yaw))
    print('yaw', i, ': ', asin(sin_yaw))

    longitude_pre = longitude_pre_list[i] + Compute.km_lo(
        (velocity_series[i] * cos_yaw * dt), latitude_series[i])
    latitude_pre = latitude_pre_list[i] + Compute.km_la(
        (velocity_series[i] * sin_yaw * dt))

    longitude_pre_list.append(longitude_pre)
    latitude_pre_list.append(latitude_pre)
print('end')

print('longitude rmse: ', Compute.root_mean_square_error(longitude_series, longitude_pre_list))
print('latitude rmse', Compute.root_mean_square_error(latitude_series, latitude_pre_list))

plt.scatter(longitude_series, latitude_series, label='measure')
plt.scatter(longitude_pre_list, latitude_pre_list, label='predict')
plt.legend()
plt.show()


