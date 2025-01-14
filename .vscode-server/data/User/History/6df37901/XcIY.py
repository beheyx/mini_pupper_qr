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

# def squat():
#     config = Configuration()
#     hardware_interface = HardwareInterface()
#     disp = Display()
#     #disp.show_ip()
#     # Create imu handle
#     imu = IMU(port="/dev/ttyACM0") if use_imu else None
#     if imu:
#         imu.flush_buffer()

#     # Create controller and user input handles
#     controller = Controller(
#         config,
#         four_legs_inverse_kinematics,
#     )
#     state = State()

#     active = True
#     command = Command()
#     dance_active_state = False
#     movementCtl = None
#     lib_length = 0
#     last_loop = time.time()
#     Move = MovementGroups()
#     Move.move_forward()
#     print("Pupper back to default")
#     dance_active_state, lib_length, movementCtl = perform_action(command, Move)
#     now = time.time()
#     if now - last_loop < config.dt:
#         break
#     last_loop = time.time()
#     # Read imu data. Orientation will be None if no data was available
#     quat_orientation = (
#         imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
#     )
#     state.quat_orientation = quat_orientation
#     # Step the controller forward by dt
#     if dance_active_state:
#         # Calculate legsLocation, attitudes and speed using custom movement script
#         movementCtl.runMovementScheme()
#         command.legslocation = movementCtl.getMovemenLegsLocation()
#         command.horizontal_velocity = movementCtl.getMovemenSpeed()
#         command.roll = movementCtl.attitude_now[0]
#         command.pitch = movementCtl.attitude_now[1]
#         command.yaw = movementCtl.attitude_now[2]
#         command.yaw_rate = movementCtl.getMovemenTurn()
#         controller.run(state, command, disp)
#         if movementCtl.movement_now_number >= lib_length - 1 and movementCtl.tick >= movementCtl.now_ticks:
#             print("Gesture completed")
#             dance_active_state = False
#     # Update the pwm widths going to the servos
#     hardware_interface.set_actuator_postions(state.joint_angles)

def setup(use_imu=False):
        """Main program"""
    #squat()
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

def action():
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

    # Update the pwm widths going to the servos
    hardware_interface.set_actuator_postions(state.joint_angles)

def squat(use_imu=False):
    """Main program"""
    #squat()
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
            user_request = input("Please enter the command: ")
           
            if "stop" in user_request :
                Move = MovementGroups()
                Move.stop()
                active = False
                print("Pupper Stopped")
                break

        # ------ Level 1 gestures --------

            # Basic movements
            elif user_request == "e":
                Move = MovementGroups()
                Move.stop()
                print("Pupper back to default")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "w":
                Move = MovementGroups()
                Move.move_forward()
                print("Pupper moves forward")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)
                break

            elif user_request == "s":
                Move = MovementGroups()
                Move.move_backward()
                print("Pupper moves backward")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "a":
                Move = MovementGroups()
                Move.rotate(10)
                print("Pupper turns left")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "d":
                Move = MovementGroups()
                Move.rotate(-10)
                print("Pupper turns right")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            # Looking
            elif user_request == "look down":
                Move = MovementGroups()
                Move.look_down()
#                Move.move_forward()
                print("Pupper looks down")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "look up":
                Move = MovementGroups()
                Move.look_up()
                print("Pupper looks up")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "look right":
                Move = MovementGroups()
                Move.look_right()
                print("Pupper looks right")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "look left":
                Move = MovementGroups()
                Move.look_left()
                print("Pupper looks left")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "look upperright":
                Move = MovementGroups()
                Move.look_upperright()
                print("Pupper looks upperright")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "look upperleft":
                Move = MovementGroups()
                Move.look_upperleft()
                print("Pupper looks upperleft")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "look rightlower":
                Move = MovementGroups()
                Move.look_rightlower()
                print("Pupper looks rightlower")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "look leftlower":
                Move = MovementGroups()
                Move.look_leftlower()
                print("Pupper looks leftlower")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            # Moving
            elif user_request == "move rightfront":
                Move = MovementGroups()
                Move.move_rightfront()
                print("Pupper moves rightfront")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "move leftfront":
                Move = MovementGroups()
                Move.move_leftfront()
                print("Pupper moves leftfront")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "move rightback":
                Move = MovementGroups()
                Move.move_rightback()
                print("Pupper moves rightback")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "move leftback":
                Move = MovementGroups()
                Move.move_leftback()
                print("Pupper moves lefitback")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)


        # ------ Level 2 gestures: pass in parameters -------

            elif user_request == "head move":
                pitch_degree = float(input("Enter the pitch degree: "))
                yaw_degree = float(input("Enter the yaw_degree: "))
                Move = MovementGroups()
                Move.head_move(pitch_degree, yaw_degree)
                print("Pupper head moves")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "body row":
                roll_degree = float(input("Enter the roll degree (+ for body moving counterclockwise, - for clockwise): "))
                Move = MovementGroups()
                Move.body_row(roll_degree)
                print("Pupper does a body row")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "height move":
                height = float(input("Enter height in meters (0.3 max, -0.2 min): "))
                Move = MovementGroups()
                Move.height_move(height)
                print("Pupper moves its height")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            elif user_request == "foreleg lift":
                foreleg = input("Type in the foreleg ('left' for left, 'right' for right): ")
                foreleg_height = float(input("Enter height in meters (positive num only): "))
                Move = MovementGroups()
                Move.foreleg_lift(foreleg, foreleg_height)
                print("Pupper lifts its foreleg")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)


            elif user_request == "backleg lift":
                backleg = input("Type in the backleg ('left' for left, 'right' for right): ")
                backleg_height = float(input("Enter height in meters (positive num only): "))
                Move = MovementGroups()
                Move.backleg_lift(backleg, backleg_height)
                print("Pupper lifts its foreleg")
                dance_active_state, lib_length, movementCtl = perform_action(command, Move)

            else:
                print("Invalid command. Pupper is confused")
                Move = MovementGroups()
                Move.body_row(30)
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

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)

#main()

def look_up(use_imu=False):
    setup(use_imu)
    
    # For specific command instructions, please refer to the doc: 
    # https://docs.google.com/document/d/1xc1WkfSX828QCnA2nI6tnsPFj9COvU4_LiW9UnFqFNo/edit?usp=sharing

    while active:
        if not dance_active_state:
            Move = MovementGroups()
            Move.look_up()
            print("Pupper looks up")
            dance_active_state, lib_length, movementCtl = perform_action(command, Move)
             
        action()
