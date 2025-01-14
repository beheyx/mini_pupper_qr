import numpy as np
import time
import sys
sys.path.append('/home/ubuntu/StanfordQuadruped')
from src.IMU import IMU
from src.Controller import Controller
from src.State import State
from MangDang.mini_pupper.HardwareInterface import HardwareInterface
from MangDang.mini_pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics
from MangDang.mini_pupper.display import Display
from src.MovementScheme import MovementScheme
from src.MovementGroup import MovementGroups
from src.Command import Command


# Helper function to shorten up the deplicated code
# command: takes in Command class which holds the state and instruction
# Move: takes in MovementGroups class to access different movement lib
def perform_action(command, Move):
    MovementLib = Move.MovementLib
    movementCtl = MovementScheme(MovementLib)  # Pass a list of movement functions
    dance_active_state = True
    lib_length = len(MovementLib)
    command.pseudo_dance_event = True
    return dance_active_state, lib_length, movementCtl

def move_forward(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.gait_uni(0.15, 0, 1, 1)
            print("Pupper moves forward")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False
        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def move_backward(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            #Move.move_backward()
            Move.gait_uni(-0.15, 0, 1, 1)
            print("Pupper moves backward")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def move_left(turn_rad, use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.gait_uni(0, 0.15, 1, 1)
            print("Pupper turns left")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)

             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def move_right(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            #Move.rotate(-10)
            Move.gait_uni(0, -0.15, 1, 1)
            print("Pupper turns right")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)

             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)


def turn_right(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.rotate(-15)
            print("Pupper moves forward")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False
        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def stop(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.stop()
            print("Pupper moves forward")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False
        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)


def turn_left(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.rotate(15)
            print("Pupper moves forward")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False
        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def look_up(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.look_up()
            print("Pupper looks up")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def look_down(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.look_down()
            print("Pupper looks down")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
            
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def look_left(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.look_left()
            print("Pupper looks left")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False
        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def look_upperleft(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.look_upperleft()
            print("Pupper looks upperleft")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def look_leftlower(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.look_leftlower()
            print("Pupper looks leftlower")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def look_right(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.look_right()
            print("Pupper looks right")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def look_upperright(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.look_upperright()
            print("Pupper looks upperright")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

def look_rightlower(use_imu=False):
    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0") if use_imu else None
    if imu:
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    active = True
    command = Command()
    dance_active_state = False
    movementCtl = None
    lib_length = 0
    last_loop = time.time()
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.look_rightlower()
            print("Pupper looks rightlower")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        if dance_active_state:
            # Calculate legsLocation, attitudes and speed using custom movement script
            movementCtl.runMovementScheme()
            command.legslocation = movementCtl.getMovemenLegsLocation()
            command.horizontal_velocity = movementCtl.getMovemenSpeed()
            command.roll = movementCtl.attitude_now[0]
            command.pitch = movementCtl.attitude_now[1]
            command.yaw = movementCtl.attitude_now[2]
            command.yaw_rate = movementCtl.getMovemenTurn()
            controller.run(state, command, disp)

            if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
                print("Gesture completed")
                dance_active_state = False
                active = False

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

