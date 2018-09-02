from numpy import array


def hx_gps(x):
    """
    state x --> GPS measurements (x,y)
    :param x: state(x,y,yaw)
    :return: h(x) GPS measurements(x,y)
    """
    return array([[x[0, 0]],
                  [x[1, 0]]])


def H_gps(x):
    """
    GPS measure H
    :param x: state(longitude,latitude,yaw)
    :return: H matrix
    """
    return array([[1, 0, 0],
                  [0, 1, 0]])

