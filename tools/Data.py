import pandas as pd

from tools import Compute


''' load data
'''


def load_data(data_length):
    data_frame = pd.read_table('../data.txt', header=0, delim_whitespace=True, nrows=data_length)
    second_series = data_frame.loc[:, '秒数']
    velocity_series = Compute.kmh_kms(data_frame.loc[:, '实际车速km/h'])   # kms

    steering_wheel_angle_series = data_frame.loc[:, '当前转角°']    # 方向盘转角,degree
    # 15000, 26000
    # 头尾较好，中间波折
    # steering_wheel_angle_series -= 1.3
    # wheel_steering_angle_series = Compute.deg_rad(-steering_wheel_angle_series/17)   # 前轮转角，rad
    # 开头较好
    steering_wheel_angle_series -= 1.21
    wheel_angle_series = Compute.deg_rad(-steering_wheel_angle_series/21)   # 前轮转角，rad

    longitude_series = data_frame.loc[:, '经度°']
    latitude_series = data_frame.loc[:, '纬度°']
    return second_series, velocity_series, wheel_angle_series, longitude_series, latitude_series


def load_noise_data(data_length):
    data_frame = pd.read_table('../data.txt', header=0, delim_whitespace=True, nrows=data_length)
    second_series = data_frame.loc[:, '秒数']
    velocity_series = Compute.kmh_kms(data_frame.loc[:, '实际车速km/h'])  # kms

    # 方向盘转角-->前轮转角
    steering_wheel_angle_series = data_frame.loc[:, '当前转角°']  # 方向盘转角,degree
    # 15000, 26000
    # 头尾较好，中间波折
    # steering_wheel_angle_series -= 1.3
    # wheel_steering_angle_series = Compute.deg_rad(-steering_wheel_angle_series/17)   # 前轮转角，rad
    # 开头较好
    steering_wheel_angle_series -= 1.21
    wheel_angle_series = Compute.deg_rad(-steering_wheel_angle_series / 21)  # 前轮转角，rad

    longitude_series = data_frame.loc[:, '经度°']
    latitude_series = data_frame.loc[:, '纬度°']
    # 模拟错误GPS信号
    latitude_series[500:800] += 0.001
    return second_series, velocity_series, wheel_angle_series, longitude_series, latitude_series


def load_wheel_data(data_length, wheelbase=0.003):
    data_frame = pd.read_table('../data.txt', header=0, delim_whitespace=True, nrows=data_length)
    second_series = data_frame.loc[:, '秒数']
    velocity_series = Compute.kmh_kms(data_frame.loc[:, '实际车速km/h'])   # kms

    front_left_wheel_velocity_series = Compute.kmh_kms(data_frame.loc[:, 'FL'])  # kms
    front_right_wheel_velocity_series = Compute.kmh_kms(data_frame.loc[:, 'FR'])
    front_wheel_steer_velocity_s = (-front_right_wheel_velocity_series + front_left_wheel_velocity_series) / wheelbase

    rear_left_wheel_velocity_series = Compute.kmh_kms(data_frame.loc[:, 'RL'])
    rear_right_wheel_velocity_series = Compute.kmh_kms(data_frame.loc[:, 'RR'])
    rear_wheel_steer_velocity_s = (-rear_right_wheel_velocity_series + rear_left_wheel_velocity_series) / wheelbase

    car_steering_velocity_series = (front_wheel_steer_velocity_s + rear_wheel_steer_velocity_s) / 2

    longitude_series = data_frame.loc[:, '经度°']
    latitude_series = data_frame.loc[:, '纬度°']
    return second_series, velocity_series, car_steering_velocity_series, longitude_series, latitude_series

