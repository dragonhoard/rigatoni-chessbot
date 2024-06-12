from enum import Enum
import time
import serial
import numpy as np
import matplotlib.pyplot as plt

from RigatoniGame import RigatoniGame, Color, Interactor
import RigatoniParameterClasses as par
from RigatoniTrajectoryGenerator import create_path_points, generate_time_trajectory, EE_Operation, TrajectoryHolder, \
    generate_time_trajectory_joint_space
from PID import PIDPositionController
from InverseDynamics import InverseDynamicsController

from dynamixel_utils.dc_motor_model import DCMotorModel
from dynio.dynamixel_controller import DynamixelIO, DynamixelMotor
from dynamixel_utils.motor import (
    get_mx28_control_table_json_path,
    group_sync_write,
    group_sync_read,
)

# open serial port for gripper communication via arduino, uncomment when using gripper
ardSerialInst = serial.Serial()
portVar = "COM7"
ardSerialInst.baudrate = 9600
ardSerialInst.port = portVar
ardSerialInst.open()
time.sleep(2)

ee_dynamixel_id = 2
# test controllers
serial_port_name = "COM5"
dxl_io = DynamixelIO(device_name=str(serial_port_name), baud_rate=57_600)

ee_motor = DynamixelMotor(
    ee_dynamixel_id,
    dxl_io,
    json_file=get_mx28_control_table_json_path(),
    protocol=2,
)


def close_gripper():
    global ardSerialInst
    print("Closing")
    command = "CLOSE"
    ardSerialInst.write(command.encode('utf-8'))
    time.sleep(1)
    return None

def open_gripper():
    global ardSerialInst
    print("Opening")
    command = "OPEN"
    ardSerialInst.write(command.encode('utf-8'))
    time.sleep(1)
    return None

down_pos = 587
up_pos = 1700
kind_of_down = 300

open_gripper()
time.sleep(.5)
ee_motor.torque_enable()
ee_motor.write_control_table("Goal_Position", down_pos)
time.sleep(1)
close_gripper()
time.sleep(1)
ee_motor.write_control_table("Goal_Position", up_pos)
time.sleep(2)
ee_motor.torque_disable()


