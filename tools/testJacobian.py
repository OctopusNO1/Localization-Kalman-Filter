import sympy
from sympy.abc import alpha, x, y, v, w, R, theta
from sympy import symbols, Matrix
from numpy import array
import numpy as np


# px, py = symbols('p_x, p_y')
# # h of landmark: (x,y,yaw)-->(range,bearing)
# z = Matrix([[sympy.sqrt((px-x)**2 + (py-y)**2)],
#             [sympy.atan2(py-y, px-x) - theta]])
# # H jacobian
# print(z.jacobian(Matrix([x, y, theta])))


# 定义符号
a, x, y, v, w, theta, time = symbols(
    'a, x, y, v, w, theta, t')
d = v * time    # km
beta = (d / w) * sympy.tan(a)   # rad
r = w / sympy.tan(a)    # km

r_lo = r / 111 / sympy.cos(y/180*np.pi)
r_la = r / 111

# 定义predict转移函数h形式，单位
fxu = Matrix(
    [[x - r_lo * sympy.sin(theta) + r_lo * sympy.sin(theta + beta)],
     [y + r_la * sympy.cos(theta) - r_la * sympy.cos(theta + beta)],
     [theta + beta]])
print(fxu)
