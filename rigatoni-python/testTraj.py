from enum import Enum
import time
import serial
from RigatoniGame import RigatoniGame, Color, Interactor
import RigatoniParameterClasses as par
from RigatoniTrajectoryGenerator import create_path_points, generate_time_trajectory, EE_Operation

# arm parameters
l1 = 0.165  # m, link 1 length
l2 = 0.165  # m, link 2 length
rest_position = [0.05, 0]

# board parameters
L = 0.280  # m, outer board side length
l = 0.256  # m, playing area side length
h = 0.052  # m, m, offset from side of board to robot base

# trajectory settings
sampling_rate = 0.001
ee_speed = 0.25
ee_acceleration = 3

# wrap all parameters
trajSet = par.TrajectorySettings(sampling_rate, ee_speed, ee_acceleration)
armRigatoni = par.Arm(l1, l2, rest_position)
boardRigatoni = par.Board(L, l, h)

# generate a fake trajectory
arm_move = 'h8h3' #'a1a5'
path_points, point_action, piece_associated = create_path_points(arm_move, 0, 0, armRigatoni, boardRigatoni)
print(path_points)
i = 0 # which trajectory
tvec, tq, tg = generate_time_trajectory(path_points[:,i], path_points[:,i+1], armRigatoni, trajSet)
print(tvec)
print(tg.q)

q = tg.q
qd = tg.qd
qdd = tg.qdd