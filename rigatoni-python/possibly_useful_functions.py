import numpy as np

# MATLAB Calculated functions
# Inertial Matrix B(q)
# last updted 6/7/2024 with finl inertias
def b_function(q):  # calculated with MATLAB
    t3 = np.cos(q[1])
    t4 = t3 * 1.478076134136031e-2
    t5 = t4 + 2.636931145600954e-2
    B = np.array([t3 * 2.956152268272062e-2 + 3.855646829783084e-2, t5, t5, 2.636931145600954e-2])
    B = B.reshape(2, 2)
    return B

# Christoffel Symbols C(q, qd)
def c_function(q, qd):
    t3 = np.sin(q[1])
    t4 = t3 * qd[0] * 1.478076134136031e-2
    t5 = t3 * qd[1] * 1.478076134136031e-2
    t6 = -t5
    C = np.array([t6, t4, -t4 + t6, 0.0])
    C = C.reshape([2, 2])
    return C

