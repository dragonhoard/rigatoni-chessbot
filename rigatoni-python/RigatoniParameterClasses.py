# classes for arm, board, pieces, controller
# to be used as parameter structs
import numpy as np
import matplotlib.pyplot as plt

def computeNeglect(value):
    # round down small imaginary or small values
    tiny = 1e-6
    if abs(np.imag(value)) < tiny:
        value = np.real(value)
    if abs(value) < tiny:
        value = 0
    return value


# class Board
class Board():
    def __init__(self, L, l, h):
        self.L = L  # outer board side length
        self.l = l  # playing area side length
        self.b = (L-l)/2  # border width
        self.s = l/8  # playing square side length

        self.h = h  # offset from side of board to origin (robot base)
        self.gv = 1.5*self.s  # offset of grave from side of board, using 1.5*s

    def draw(self, ax):
        # plot base joint
        ax.plot(0, 0, marker='o')
        # plot outer border
        pts = np.array([[-self.L/2, self.h],
                [-self.L/2, self.h + self.L],
                [self.L/2, self.h + self.L],
                [self.L/2, self.h],
                [-self.L/2, self.h]])
        ax.plot(pts[:, 0], pts[:, 1], ls='-', color='black')

        # plot horizontal lines
        x_min = -self.l/2
        x_max =  self.l/2
        for i in range(9):
            y = self.h + self.b + i*self.s
            ax.hlines(y, x_min, x_max, ls='-', color='black')

        # plot vertical lines
        y_min = self.h + self.b
        y_max = self.h + self.b + self.l
        for i in range(-4,5):
            x = i*self.s
            ax.vlines(x, y_min, y_max, ls='-', color='black')

        # plot grave spots
        gvsptx = self.L/2 + self.gv
        gvspty = self.h + self.L/2
        ax.plot(gvsptx, gvspty, marker='x', color='black')
        ax.plot(-gvsptx, gvspty, marker='x', color='black')

    def get_grave(self, who, perspective):
        # who indicate whose grave to find, white (0) or black (1)
        # perspective indicate looking from which side (if origin is on white (0) or black (1) side
        flip = who + perspective
        x = (self.L/2 + self.gv) * (-1)**flip
        y = self.L/2 + self.h

        pos = np.array([x, y])
        pos.shape = (2, 1)
        prepos = np.array([x-self.s*(-1)**flip, y])
        prepos.shape = (2, 1)
        # pos is grave location
        # prepos is lower down position to push last killed piece away
        return prepos, pos

    def get_square_position(self, square, perspective):
        # perspective indicate looking from which side (if origin is on white (0) or black (1) side
        def letter2num(letter):
            return float(ord(letter) - ord('a') + 1)
        col = letter2num(square[0])
        row = float(square[1])

        if perspective == 1: # if looking from other side of board, flip
            row = 9 - row
            col = 9 - col

        # calculate position of middle of a1 square (11)
        a1x = -(self.l/2 - self.s/2)
        a1y = self.h + self.b + self.s/2

        x = a1x + (col-1)*self.s
        y = a1y + (row-1)*self.s

        pos = np.array([x, y])
        pos.shape = (2,1)

        return pos



# class Arm, performs forward/inverse kinematics and plotting
class Arm():

    def __init__(self, l1, l2, rest_position):
        self.l1 = l1
        self.l2 = l2
        self.rest_position = np.zeros([2,1])
        self.rest_position[0] = rest_position[0]
        self.rest_position[1] = rest_position[1]

    def FK(self, q):
        # end pos from joint position
        th1 = q[0]
        th2 = q[1]

        x_e = np.zeros([2, 1])
        x_e[0] = self.l1 * np.cos(th1) + self.l2 * np.cos(th1 + th2)
        x_e[1] = self.l1 * np.sin(th1) + self.l2 * np.sin(th1 + th2)
        x_e.shape = (2,1)

        return x_e

    def IK(self, x_e, choice=0):
        q = np.zeros(2,)

        # compute th2
        c2 = computeNeglect((x_e[0] ** 2 + x_e[1] ** 2 - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2))
        s2 = computeNeglect(np.sqrt(1 - c2 ** 2))

        if choice == 1:
            s2 = -s2
        q[1] = np.arctan2(s2, c2)

        # compute th1
        a = self.l1 + self.l2 * c2
        b = self.l2 * s2

        q[0] = np.arctan2(a * x_e[1] - x_e[0] * b, x_e[0] * a + b * x_e[1])

        # combine into q
        return q

    def Jacobian(self, q):
        s1 = np.sin(q[0])
        c1 = np.cos(q[0])
        s12 = np.sin(q[0] + q[1])
        c12 = np.cos(q[0] + q[1])

        J = np.zeros([2, 2])
        J[0, 0] = - self.l1 * s1 - self.l2 * s12
        J[0, 1] = - self.l2 * s12
        J[1, 0] = self.l1 * c1 + 2 * c12
        J[1, 1] = self.l2 * c12

        return J

    def plot_arm(self, q, ax):
        jt1 = np.array([0, 0])
        jt2 = self.l1 * np.array([np.cos(q[0]), np.sin(q[0])])
        jt3 = jt2 + self.l2 * np.array([np.cos(q[0] +q[1]), np.sin(q[0] +q[1])])
        pts = np.stack([jt1, jt2, jt3])

        ax.plot(pts[:, 0], pts[:, 1], ls='o-')

    def plot_ee(self, q, ax):
        ee = self.FK(q)
        ax.plot(ee[0], ee[1], ls='*')


class TrajectorySettings():
    def __init__(self, sampling_rate, ee_speed, ee_accel, buffer = 0):
        self.sampling_rate = sampling_rate
        self.ee_speed = ee_speed
        self.ee_acceleration = ee_accel
        self.buffer = buffer




