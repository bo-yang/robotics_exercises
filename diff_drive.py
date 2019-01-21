import numpy as np


def diffdrive(x, y, theta, v_l, v_r, t, l):
    """ Implement the forward kinematics for the differential drive.
        x, y, theta is the pose of the robot.
        v_l and v_r are the speed of the left and right wheels.
        t is the driving time.
        l is the distance between the wheels.
    """
    if v_l == v_r:
        theta_n = theta
        x_n = x + v_l * t * np.cos(theta)
        y_n = y + v_l * t * np.sin(theta)
    else:
        R = l/2 * (v_l + v_r) / (v_r - v_l)
        ICC_x = x - R * np.sin(theta)
        ICC_y = y + R * np.cos(theta)
        omega = (v_r - v_l) / l
        ol = omega * t
        Tf = np.array([[np.cos(ol), -np.sin(ol), 0], [np.sin(ol), np.cos(ol), 0], [0, 0, 1]])
        X = np.array([x-ICC_x, y-ICC_y, theta])
        v = np.array([ICC_x, ICC_y, ol])
        (x_n, y_n, theta_n) = np.matmul(Tf, X) + v
    return x_n, y_n, theta_n  # new pose of the robot


L = 0.5
x = 1.5
y = 2.0
theta = np.pi/2
(x_1, y_1, theta_1) = diffdrive(x, y, theta, 0.3, 0.3, 3, L)
(x_2, y_2, theta_2) = diffdrive(x_1, y_1, theta_1, 0.1, -0.1, 1, L)
(x_3, y_3, theta_3) = diffdrive(x_2, y_2, theta_2, 0.2, 0, 2, L)
