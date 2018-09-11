import pandas as pd
from numpy import tan, pi
import random

from tools import Compute


''' load data
'''


def load_data(data_length, correction=1.21, conversion=21, data_name='../data.txt'):
    """

    :param data_length:
    :param correction: 修正
    :param conversion: 方向盘转角-->前轮转角
    :param data_name:
        头尾较好，中间（15000, 26000）波折：1.3和17
        开头较好：1.21和21
    :return:
    """
    data_frame = pd.read_table(data_name, header=0, delim_whitespace=True, nrows=data_length)
    second_series = data_frame.loc[:, '秒数']
    velocity_series = Compute.kmh_kms(data_frame.loc[:, '实际车速km/h'])   # kms

    steering_wheel_angle_series = data_frame.loc[:, '当前转角°']    # 方向盘转角,degree
    steering_wheel_angle_series -= correction   # 修正
    wheel_angle_series = Compute.deg_rad(-steering_wheel_angle_series/conversion)   # 前轮转角，rad

    longitude_series = data_frame.loc[:, '经度°']
    latitude_series = data_frame.loc[:, '纬度°']
    return second_series, velocity_series, wheel_angle_series, longitude_series, latitude_series


def load_noise_data(data_length, correction=1.21, conversion=21, data_name='../data.txt'):
    data_frame = pd.read_table(data_name, header=0, delim_whitespace=True, nrows=data_length)
    second_series = data_frame.loc[:, '秒数']
    velocity_series = Compute.kmh_kms(data_frame.loc[:, '实际车速km/h'])  # kms

    # 方向盘转角-->前轮转角
    steering_wheel_angle_series = data_frame.loc[:, '当前转角°']  # 方向盘转角,degree
    steering_wheel_angle_series -= correction
    wheel_angle_series = Compute.deg_rad(-steering_wheel_angle_series / conversion)  # 前轮转角，rad

    longitude_series = data_frame.loc[:, '经度°']
    latitude_series = data_frame.loc[:, '纬度°']
    # 模拟错误GPS信号：
    # 1、纬度偏离
    # latitude_series[6000:9000] += 0.001
    # 2、逐渐偏离
    # for i in range(2000):
    #     latitude_series[12000+i] -= 0.0000005*i
    # latitude_series[14000:16000] -= 0.001
    # for i in range(1000):
    #     latitude_series[16000+i] -= 0.000001*(1000-i)
    # 3、noise
    for i in range(3000):
        latitude_series[12000+i] += random.gauss(0, 0.0001)
    return second_series, velocity_series, wheel_angle_series, longitude_series, latitude_series


def load_wheel_data(data_length, wheelbase=0.0015, data_name='../data.txt'):
    data_frame = pd.read_table(data_name, header=0, delim_whitespace=True, nrows=data_length)
    second_series = data_frame.loc[:, '秒数']
    # velocity_series = Compute.kmh_kms(data_frame.loc[:, '实际车速km/h'])   # kms

    front_left_wheel_velocity_series = Compute.kmh_kms(data_frame.loc[:, 'FL'])  # kms
    front_right_wheel_velocity_series = Compute.kmh_kms(data_frame.loc[:, 'FR'])
    rear_left_wheel_velocity_series = Compute.kmh_kms(data_frame.loc[:, 'RL'])
    rear_right_wheel_velocity_series = Compute.kmh_kms(data_frame.loc[:, 'RR'])

    velocity_series = (front_left_wheel_velocity_series + front_right_wheel_velocity_series +
                       rear_left_wheel_velocity_series + rear_right_wheel_velocity_series) / 4

    front_wheel_steer_velocity_s = (front_right_wheel_velocity_series - front_left_wheel_velocity_series) / wheelbase
    # car_steering_velocity_series = front_wheel_steer_velocity_s
    rear_wheel_steer_velocity_s = (rear_right_wheel_velocity_series - rear_left_wheel_velocity_series) / wheelbase
    # car_steering_velocity_series = rear_wheel_steer_velocity_s
    car_steering_velocity_series = (front_wheel_steer_velocity_s + rear_wheel_steer_velocity_s) / 2

    longitude_series = data_frame.loc[:, '经度°']
    latitude_series = data_frame.loc[:, '纬度°']
    return second_series, velocity_series, car_steering_velocity_series, longitude_series, latitude_series


def load_process_data(data_length, correction=1.21, conversion=21, wheelbase=0.003):
    """
    IMU bicycle：车辆转角<--前轮转角<--方向盘转角
        用PF修正错误的bicycle预测
    GPS：arctan(dy/dx)
    dt时间差，0.02
    :param data_length:
    :param correction: 修正
    :param conversion: 方向盘转角-->前轮转角
        头尾较好，中间（15000, 26000）波折：1.3和17
        开头较好：1.21和21
    :param wheelbase: 0.003km
    :return:
    """
    data_frame = pd.read_table('../data.txt', header=0, delim_whitespace=True, nrows=data_length)

    second_series = data_frame.loc[:, '秒数']
    velocity_series = Compute.kmh_kms(data_frame.loc[:, '实际车速km/h'])   # km/s
    longitude_series = data_frame.loc[:, '经度°']
    latitude_series = data_frame.loc[:, '纬度°']
    steering_wheel_angle_series = data_frame.loc[:, '当前转角°']    # 方向盘转角,degree
    steering_wheel_angle_series -= correction   # 修正
    wheel_angle_series = Compute.deg_rad(-steering_wheel_angle_series/conversion)   # 前轮转角，rad

    dt_series, turn_angle_series = [], []
    for i in range(len(second_series)-1):
        dt = second_series[i+1] - second_series[i]
        dt_series.append(dt)     # s

        dist = velocity_series[i] * dt_series[i]   # km
        turn_angle_series.append((dist / wheelbase) * tan(wheel_angle_series[i]))
    # 补一个没用的
    dt_series.append(0.02)
    turn_angle_series.append(0)
    return dt_series, turn_angle_series, velocity_series, longitude_series, latitude_series

