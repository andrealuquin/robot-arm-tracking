import numpy as np


def forward_kinematics(q, link_lengths):
    q1, q2, q3 = q
    l1, l2, l3 = link_lengths

    r = l2 * np.cos(q2) + l3 * np.cos(q2 + q3)
    x = np.cos(q1) * r
    y = np.sin(q1) * r
    z = l1 + l2 * np.sin(q2) + l3 * np.sin(q2 + q3)

    return np.array([x, y, z])


def jacobian(q, link_lengths):
    q1, q2, q3 = q
    l1, l2, l3 = link_lengths

    r = l2 * np.cos(q2) + l3 * np.cos(q2 + q3)

    dx_dq1 = -np.sin(q1) * r
    dx_dq2 = np.cos(q1) * (-l2 * np.sin(q2) - l3 * np.sin(q2 + q3))
    dx_dq3 = np.cos(q1) * (-l3 * np.sin(q2 + q3))

    dy_dq1 = np.cos(q1) * r
    dy_dq2 = np.sin(q1) * (-l2 * np.sin(q2) - l3 * np.sin(q2 + q3))
    dy_dq3 = np.sin(q1) * (-l3 * np.sin(q2 + q3))

    dz_dq1 = 0.0
    dz_dq2 = l2 * np.cos(q2) + l3 * np.cos(q2 + q3)
    dz_dq3 = l3 * np.cos(q2 + q3)

    return np.array([
        [dx_dq1, dx_dq2, dx_dq3],
        [dy_dq1, dy_dq2, dy_dq3],
        [dz_dq1, dz_dq2, dz_dq3],
    ])
