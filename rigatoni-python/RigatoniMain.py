from enum import Enum
import time
import serial
from RigatoniGame import RigatoniGame, Color, Interactor
import RigatoniParameterClasses as par
from RigatoniTrajectoryGenerator import create_path_points, generate_time_trajectory, EE_Operation

engine_path = r"C:\Users\daced\Downloads\fairy-stockfish-largeboard_x86-64-bmi2.exe"

# open serial port for gripper communication via arduino
'''
ardSerialInst = serial.Serial()
portVar = "COM7"
ardSerialInst.baudrate = 9600
ardSerialInst.port = portVar
ardSerialInst.open()
'''

def chosen_controller():
    return None

def ee_commander(height):
    return None
def close_gripper():
    global ardSerialInst
    print("Closing")
    command = "CLOSE"
    #ardSerialInst.write(command.encode('utf-8'))
    time.sleep(2)
    return None
def open_gripper():
    global ardSerialInst
    print("Opening")
    command = "OPEN"
    #ardSerialInst.write(command.encode('utf-8'))
    time.sleep(2)
    return None

# arm parameters
l1 = 0.156  # m, link 1 length
l2 = 0.156  # m, link 2 length
rest_position = [0.05, 0]

# board parameters
L = 0.280  # m, outer board side length
l = 0.256  # m, playing area side length
h = 0.052  # m, m, offset from side of board to robot base

# trajectory settings
sampling_rate = 0.001
ee_speed = 0.25
ee_acceleration = 3

trajSet = par.TrajectorySettings(sampling_rate, ee_speed, ee_acceleration)
armRigatoni = par.Arm(l1, l2, rest_position)
boardRigatoni = par.Board(L, l, h)
Rigatoni_side = Color.WHITE  # which side is Rigatoni installed on

def robot_commander(arm_move, ee_info, turn):
    global armRigatoni
    global boardRigatoni
    global Rigatoni_side
    # turn, whose turn

    # get pathpoints and EE instructions
    path_points, point_action, piece_associated = create_path_points(arm_move, turn, Rigatoni_side, armRigatoni, boardRigatoni)

    # for each pathpoint
    for i in range(len(path_points[1,:])):
    #   perform EE instruction at pathpoint
        match point_action[i]:
            case EE_Operation.NOTHING:
                print('ee does nothing')
                pass
            case EE_Operation.PICK_UP:
                print('pick up')
                # get piece pickup height, clearance height, lower height, close diameter
                #grip_height, clearance, lower, closed_dia = ChessSet.get_operation_dimensions(piece_associated[i])
                #ee_commander(grip_height)
                time.sleep(.1)
                close_gripper()
                time.sleep(.1)
                #ee_commander(clearance)
            case EE_Operation.PUT_DOWN:
                print('put down')
                #grip_height, clearance, lower, closed_dia = ChessSet.get_operation_dimensions(piece_associated[i])
                #ee_commander(grip_height)
                time.sleep(.1)
                open_gripper()
                time.sleep(.1)
                #ee_commander(ChessSet.clearance_height)
            case EE_Operation.LOWER:
                print('lower')
                #grip_height, clearance, lower, closed_dia = ChessSet.get_operation_dimensions(piece_associated[i])
                #ee_commander(lower)
                print(path_points)
                print(point_action)

        if not i == len(path_points[1,:]) - 1: # if not last point
            print('generate trajectory')
            #   generate trajectory to next pathpoint
            tvec, tq, tg = generate_time_trajectory(path_points[:,i], path_points[:,i+1], armRigatoni, trajSet)
            print(tg)
            #   feed trajectory to chosen controller
            chosen_controller()


## GAME SETTINGS
# define who decides the move, and who physically makes the move for each player
white_info = [Interactor.BOT, Interactor.BOT ]
black_info = [Interactor.BOT, Interactor.USER ]
bot_side = Color.WHITE

# start the game
game = RigatoniGame(white_info, black_info, bot_side, robot_commander, engine_path)

