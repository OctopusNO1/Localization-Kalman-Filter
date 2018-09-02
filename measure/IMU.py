from math import sin, cos, tan
import numpy as np

from tools import Compute


# predict使用的motion model
def move(x, u, dt=0.02, wheelbase=0.003):
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
