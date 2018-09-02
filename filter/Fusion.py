import numpy as np
from numpy import array
from numpy.linalg import inv
from filterpy.stats import plot_covariance_ellipse

from filter.EKF import EKF


def run_localization(std_vel, std_steer, std_lo, std_la,
                     us, zs, dt,
                     wheelbase, data_length,
                     ellipse_step=100):
    ekf_ = EKF(dt, wheelbase, std_vel=std_vel, std_steer=std_steer)
    ekf_.x = array([[117.301701, 39.116025, 3.0]]).T  # x, y, yaw
    ekf_.P = np.diag([0.001, 0.001, 11.1])    # yaw uncertain
    ekf_.R = np.diag([std_lo ** 2, std_la ** 2])    # measure uncertain
    # adapt
    Q_scale_factor = 1000.
    eps_max = 4.
    count = 0

    predicts = [ekf_.x]
    estimates = [ekf_.x]
    for i in range(data_length-1):
        # predict
        ekf_.predict(u=us[i])
        predicts.append(ekf_.x)
        # # # 画出predict的协方差椭圆
        # if i % ellipse_step == 0:
        #     plot_covariance_ellipse(
        #         (ekf_.x[0, 0], ekf_.x[1, 0]), ekf_.P[0:2, 0:2],
        #         std=0.006, facecolor='k', alpha=0.3)

        # update
        ekf_.update_gps(z=zs[i])

        # adapt: adjust Q,R by residual and measure covariance, y^2/S>threshold
        y, S = ekf_.y, ekf_.S
        eps = np.dot(y.T, inv(S)).dot(y)
        if eps > eps_max:
            ekf_.Q *= Q_scale_factor
            count += 1
        elif count > 0:
            ekf_.Q /= Q_scale_factor
            count -= 1

        estimates.append(ekf_.x)
        # 画出update后的协方差椭圆
        if i % ellipse_step == 0:
            plot_covariance_ellipse(
                (ekf_.x[0, 0], ekf_.x[1, 0]), ekf_.P[0:2, 0:2],
                std=0.006, facecolor='g', alpha=0.8)
    predicts = array(predicts)
    estimates = array(estimates)
    return ekf_, predicts, estimates
