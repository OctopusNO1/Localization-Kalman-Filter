from math import tan

from tools import Compute


sum_ = 72.255300 + 72.427200 + 72.026100 + 72.198000
mean_ = sum_ / 4
print(mean_)
print(mean_ / 3.6)

dt = 0.970333 - 0.942257

wheel_radius = 0.316
wheelbase = 3    # m
wheel_steering_angle1 = wheel_radius * (72.427200 - 72.255300) / 3.6 / wheelbase   # m/s
wheel_steering_angle2 = wheel_radius * (72.198000 - 72.026100) / 3.6 / wheelbase
print(wheel_steering_angle1)
print(wheel_steering_angle2)

steering_wheel_angle = -1.000000
wheel_steering_angle = Compute.deg_rad(-steering_wheel_angle/21)
print(wheel_steering_angle)

distance = 72.226650 / 3.6 * dt     # m
turn_angle = distance / wheelbase * tan(wheel_steering_angle)
print(turn_angle)
