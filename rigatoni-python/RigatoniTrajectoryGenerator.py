# functions for trajectory and storage

# Code written for final project of UCLA MAEC263C course
# editor: Jane
# last edit: 6/5/2024 11:51 by Jane

import numpy as np
import roboticstoolbox as rtb
import roboticstoolbox.tools.trajectory as trajectory

import time

import RigatoniParameterClasses as par


from enum import Enum

class EE_Operation(Enum):
    NOTHING = 0
    PICK_UP = 1  # lower, pickup, raise
    PUT_DOWN = 2  # lower, put down, raise
    LOWER = 3  # partial lower (for grave drops)


def create_path_points(move, player, perspective, arm, board):
    # move = letter-number combo specifying uci move and kill (k or any char at end)
    # player = player making the move
    # perspective = coordinate frame perspective (side that bot is on) 0-white, 1-black
    # arm = Arm object, used to get rest position (2x1)
    # board = Board object

    # determine start square and end square
    start_square = board.get_square_position(move[0:2], perspective)
    end_square = board.get_square_position(move[2:4], perspective)

    # first position is rest position
    path_points = arm.rest_position
    point_action = [EE_Operation.NOTHING]
    piece_associated = [0] # first piece

    # generate kill trajectory
    if len(move) == 5:  # kill instruction included
        [pregrave, grave] = board.get_grave(player ^ 1, perspective)
        path_points = np.concatenate((path_points, end_square, pregrave, grave), axis=1)
        point_action.append(EE_Operation.PICK_UP)  # pick up victim
        point_action.append(EE_Operation.LOWER)  # lower victim
        point_action.append(EE_Operation.PUT_DOWN)  # put down victim
        piece_associated.extend((1, 1, 1))

    # generate move trajectory
    path_points = np.concatenate((path_points, start_square, end_square, arm.rest_position), axis=1)
    point_action.append(EE_Operation.PICK_UP)  # pick up moving piece
    point_action.append(EE_Operation.PUT_DOWN)  # put down moving piece
    point_action.append(EE_Operation.NOTHING)
    piece_associated.extend((0, 0, 0))

    return path_points, point_action, piece_associated

def generate_time_trajectory(point_a, point_b, arm, traj_set):
    distance = np.linalg.norm(point_b - point_a)

    # estimate time for parabolic blend
    tb = traj_set.ee_speed / traj_set.ee_acceleration
    distance_covered = tb ** 2 * traj_set.ee_acceleration
    linear_time = (distance - distance_covered)/traj_set.ee_speed
    t = 2 *tb + max(0, linear_time) + traj_set.buffer

    V = traj_set.ee_speed*1.01
    while V > traj_set.ee_speed:
        t = traj_set.sampling_rate * np.ceil(t / traj_set.sampling_rate)
        tvec = np.arange(0, t+traj_set.sampling_rate, traj_set.sampling_rate)
        tg = rtb.mtraj(rtb.trapezoidal, point_a, point_b, tvec)

        t += 0.01 # if velocity is too high, will loop around and give more time
        V = np.sqrt(max(tg.q[:,0]**2 + tg.q[:,1]**2))
        if V > traj_set.ee_speed:
            tg = rtb.mtraj(rtb.trapezoidal, point_a, point_b, tvec)
        V = np.sqrt(max(tg.qd[:, 0] ** 2 + tg.qd[:, 1] ** 2))

    # convert to joint space
    q_des = np.zeros([2, len(tvec)])
    qd_des = np.zeros([2, len(tvec)])
    for i in range(len(tvec)):
        q_des[:, i] = arm.IK(np.array(tg.q[i,:]))
        qd_des[:, i] = np.linalg.inv(arm.Jacobian(q_des[:, i])) @ np.array(tg.qd[i,:])
    qdd_des = np.append(np.diff(qd_des), [[0],[0]], axis=1)

    tq = trajectory.Trajectory('mtraj', tvec, q_des.T, qd_des.T, qdd_des.T)

    return tvec, tq, tg


def generate_time_trajectory_joint_space(point_a, point_b, arm, traj_set):
    distance = np.linalg.norm(point_b - point_a)

    pos_a = arm.IK(point_a)
    pos_b = arm.IK(point_b)

    # estimate time for parabolic blend
    tb = traj_set.ee_speed / traj_set.ee_acceleration
    distance_covered = tb ** 2 * traj_set.ee_acceleration
    linear_time = (distance - distance_covered)/traj_set.ee_speed
    t = 2 *tb + max(0, linear_time)

    t = traj_set.sampling_rate * np.ceil(t / traj_set.sampling_rate)
    tvec = np.arange(0, t+traj_set.sampling_rate, traj_set.sampling_rate)
    tq = rtb.mtraj(rtb.trapezoidal, point_a, point_b, tvec)

    # # convert to joint space
    # q_des = np.zeros([2, len(tvec)])
    # qd_des = np.zeros([2, len(tvec)])
    # for i in range(len(tvec)):
    #     q_des[:, i] = arm.IK(np.array(tg.q[i,:]))
    #     qd_des[:, i] = np.linalg.inv(arm.Jacobian(q_des[:, i])) @ np.array(tg.qd[i,:])
    # qdd_des = np.append(np.diff(qd_des), [[0],[0]], axis=1)
    #
    # tq = trajectory.Trajectory('mtraj', tvec, q_des.T, qd_des.T, qdd_des.T)

    return tvec, tq



class TrajectoryHolder():
    # TrajectoryHolder assumes trajectory inquiries will only ever be requested
    # in time increasing order, unless restart_traj is called

    def __init__(self, t, q, qd, qdd):
        self.t = t
        self.end_time = t[-1]
        self.q = q
        self.qd = qd
        self.qdd = qdd
        self.idx = 0  # current time index

    def get_end_time(self):
        return self.end_time

    def get_q(self, time):
        if time >= self.end_time:  # if past end time or is end time
            return self.q[-1,:]   # return final desired posiion

        while (self.t[self.idx] < time):  # advance index to the closest timestep (overshooting)
            self.idx += 1

        return self.q[self.idx,:]

    def get_q_qd_qdd(self, time):
        if time > self.end_time:  # if past end time
            return self.q[-1,:], self.qd[-1,:], self.qdd[-1,:]  # return final values of trajectory

        while (self.t[self.idx] < time):  # advance index to the closest timestep (overshooting)
            self.idx += 1

        return self.q[self.idx,:], self.qd[self.idx,:], self.qdd[self.idx,:]

    def restart_traj(self): # in case it is ever needed to go back to th beginning of a trajectory, this resets the index
        self.idx = 0


# for testing purposes
'''
import RigatoniParameterClasses as par
arm = par.Arm(.156, .156, [0, .02])
board = par.Board(.280, .256, .0)
path_points, point_action, piece_associated = create_path_points('e1e2k', 0, 0, arm, board)
print(path_points)
print(point_action)
print(piece_associated)

traj_set = par.TrajectorySettings(.001, 0.25, 3)
tvec, tq, tg = generate_time_trajectory(np.array([0,.05]), np.array([0, .1]), arm, traj_set)

tg.plot()
tq.plot()
'''




'''
t = np.arange(0,10,.1)
print(t)
tg = rtb.trapezoidal(1, 2, t)
tg.plot()

board = par.Board(81, 80, 2)
print(board.get_square_position('a1', 0))
print(board.get_square_position('a1', 1))
print(board.get_grave(0,0))
print(board.get_grave(1,0))
print(board.get_grave(0,1))
print(board.get_grave(1,1))
'''

