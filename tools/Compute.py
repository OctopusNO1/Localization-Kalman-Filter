from math import radians, cos, sqrt, pi


''' compute
'''


def steer_w_vehicle_w(steer_w):
    # 方向盘转角-->前轮转角
    # 可能的修正：向左-1.2
    steer_w = steer_w-1.2
    # 可能的转换比：21
    vehicle_w = -steer_w/21
    return vehicle_w


def deg_rad(degree):
    return degree/180*pi


def kmh_kms(kmh):
    return kmh/3600


def km_lo(km, latitude):
    return km/111/cos(radians(latitude))


def km_la(km):
    return km/111


def cos_yaw(x, y, x_, y_):
    distance = sqrt((x_ - x)**2 + (y_ - y)**2)
    if distance == 0:   # no move
        return 0
    else:
        return (x_ - x) / distance


def sin_yaw(x, y, x_, y_):
    distance = sqrt((x_ - x) ** 2 + (y_ - y) ** 2)
    if distance == 0:  # no move
        return 0
    else:
        return (y_ - y) / distance


def root_mean_square_error(series1, series2):
    sum_ = 0
    for (i1, i2) in zip(series1, series2):
        sum_ += (i1 - i2)**2
    return sqrt(sum_/len(series1))

