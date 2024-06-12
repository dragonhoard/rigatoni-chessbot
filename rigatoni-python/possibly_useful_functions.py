import numpy as np

# MATLAB Calculated functions
# Inertial Matrix B(q)
# last updted 6/7/2024 with finl inertias
def b_function(q):  # calculated with MATLAB
    t3 = np.cos(q[1])
    t4 = t3 * 5.97310296e-3
    t5 = t4 + 4.558903792190174e-3
    B = np.array([t3 * 1.194620592e-2+1.71795253067423e-2, t5, t5, 8.772449086163049e-3])
    B = B.reshape(2, 2)
    return B

# Christoffel Symbols C(q, qd)
def c_function(q, qd):
    t3 = np.sin(q[1])
    t4 = t3 * qd[0] * 5.97310296e-3
    t5 = t3 * qd[1] * 5.97310296e-3
    t6 = -t5
    C = np.array([t6, t4, -t4 + t6, 0.0])
    C = C.reshape([2, 2])
    return C

