import numpy as np

# MATLAB Calculated functions
# Inertial Matrix B(q)
def b_function(self, q):  # calculated with MATLAB
    t3 = np.cos(q[1])
    t4 = t3 * 4.183051653e-3
    t5 = t4 + 3.86841468610732e-3
    B = np.array([t3 * 8.366103306e-3 + 1.296029878973884e-2, t5, t5, 3.86841468610732e-3])
    B = B.reshape(2, 2)
    return B

# Christoffel Symbols C(q, qd)
def c_function(self, q, qd):
    t3 = np.sin(q[1])
    t4 = t3 * qd[0] * 4.183051653e-3
    t5 = t3 * qd[1] * 4.183051653e-3
    t6 = -t5
    C = np.array([t6, t4, -t4 + t6, 0.0])
    C = C.reshape([2, 2])
    return C

