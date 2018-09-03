
import matplotlib.pyplot as plt

from tools import Data


""" GPS不准时，如何定位？ 
    使用IMU"""
data_length = 13000
_, _, _, longitude_series, latitude_series = Data.load_data(data_length)
_, _, _, noise_longitude_series, noise_latitude_series = Data.load_noise_data(data_length)


plt.scatter(longitude_series, latitude_series, label='true_measure')
plt.scatter(noise_longitude_series, noise_latitude_series, label='noise_measure')
plt.legend()
plt.show()
