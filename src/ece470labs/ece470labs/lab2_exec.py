#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
import time
import numpy as np
from math import pi
import sys
class JointAngles:
    def __init__(self):
        self.name = ["", "", "", "", "", ""]  #could have also done [""] * 6
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# UR3e home position
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 
Q11 = [124.41,  -58.06, 122.40, -154.32,    -90.06, 4.58]
Q12 = [124.42,  -66.26, 121.00, -144.72,    -90.05, 4.56]
Q13 = [124.42,  -72.60, 118.54, -135.92,    -90.04, 4.54]
Q21 = [146.89,  -64.54, 138.08, -163.43,    -90.07, 27.05]
Q22 = [146.90,  -75.26, 136.33, -150.97,    -90.05, 27.03]
Q23 = [146.90,  -83.85, 133.16, -139.21,    -90.04, 27.01]
Q31 = [173.59,  -62.79, 133.14, -160.16,    -90.01, 53.76]
Q32 = [173.59,  -72.23, 131.57, -149.15,    -89.99, 53.74]
Q33 = [173.60,  -80.35, 128.52, -137.98,    -89.98, 53.72]

T1 = [124.43,   -86.38, 101.73, -105.34,    -90.01, 4.47] # -209.96
T2 = [146.91,   -99.80, 114.27, -104.38,    -90.01, 26.94]
T3 = [173.61,   -95.40, 110.52, -104.93,    -89.95, 53.65]

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], [Q21, Q22, Q23], [Q31, Q32, Q33] ]
T = [ T1, T2, T3 ]

for i in range(0, 3):
    for j in range(0, 3):
        for k in range(0, 6):
            Q[i][j][k] = Q[i][j][k] * pi / 180.0

    for j in range(0, 6):
        T[i][j] = T[i][j] * pi / 180.0

############### Your Code End Here ###############
class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        ############## Your Code Start Here ##############
        # TODO: define a ROS subscriber for gripper input message and corresponding callback function
        # ROS2 gripper input topic: /io_and_status_controller/io_states

        self.gripper_input_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_state_callback, 10)

        ############### Your Code End Here ###############

        # Service clients
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IO service not available, waiting...')

        # State variables
        self.current_joint_state = None
        self.analog_in_0_value = 0
        self.current_JointAngles = JointAngles()
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ] # shoulder_pan_joint is the base rotation joint

    def joint_state_callback(self, msg):
        self.current_joint_state = msg  # Currently only used to check if messages have arrived
        index_inOrder = 0
        for name in self.joint_names:
            index_outofOrder = msg.name.index(name)
            self.current_JointAngles.name[index_inOrder] = name
            self.current_JointAngles.position[index_inOrder] = msg.position[index_outofOrder]
            index_inOrder = index_inOrder + 1 


    def io_state_callback(self, msg):
    ############## Your Code Start Here ##############
        """
        TODO: define a ROS topic callback funtion that 
        receives and stores the state of the suction cup
        Whenever /io_and_status_controller/io_states 
        publishes this info, this callback function is
        called.
        """

        for analog_in in msg.analog_in_states:
            if analog_in.pin == 0:
                self.analog_in_0_value = analog_in.state
                break

    ############### Your Code End Here ###############

    def set_io(self, pin, state):
        req = SetIO.Request()
        req.fun = 1
        req.pin = pin
        req.state = state
        future = self.io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def move_arm(self, target):
        if self.current_joint_state is None:
            self.get_logger().error("No joint state received!")
            return False

        V_MAX = 1#2.09    # rad/s
        A_MAX = 0.8#2.79   # rad/s^2
        MIN_DURATION = 1
        MAX_DURATION = 8.0

        deltas = []
        for i in range(6):
            deltas.append(abs(self.current_JointAngles.position[i] - target[i]))


        max_delta = max(deltas)
        t_acc = V_MAX / A_MAX
        d_acc = 0.5 * A_MAX * (t_acc ** 2)
        if max_delta > 2 * d_acc:
            # trapezoidal velocity profile
            t_total = 2 * t_acc + (max_delta - 2 * d_acc) / V_MAX
        else:
            # triangular velocity profile
            t_total = 2 * (max_delta / A_MAX) ** 0.5

        duration = max(MIN_DURATION, min(t_total, MAX_DURATION))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Start immediately when the controller receives it
        trajectory_msg.header.stamp.sec = 0
        trajectory_msg.header.stamp.nanosec = 0

        # Anchor point: current measured joint state at t = 0
        p0 = JointTrajectoryPoint()
        p0.positions = self.current_JointAngles.position
        p0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        trajectory_msg.points.append(p0)

        # Goal point
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # end at rest, 2 point trajectory
        p1.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #end at rest.
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory_msg.points.append(p1)

        self.trajectory_pub.publish(trajectory_msg)

        self.get_logger().info(f'Moving to position: {np.degrees(target)}')

        # Wait for movement completion
        start_time = time.time()
        while time.time() - start_time < duration + 2:
            rclpy.spin_once(self, timeout_sec=0.1)

            deltas = []
            for i in range(6):
                deltas.append(abs(self.current_JointAngles.position[i] - target[i]))
            if all(delta < 0.001 for delta in deltas):
                time.sleep(0.25)
                return True
        return False



    def move_block(self, start_tower, start_height, end_tower, end_height):
        global Q
        global T
        ############## Your Code Start Here ##############
        # TODO: add code to move block from start tower and height to end tower and height
        ### Hint: Use the Q array to map out your towers by location and "height".

        # Get locations of the start and destination
        start_target = Q[start_tower][start_height]
        end_target = Q[end_tower][end_height]

        # Move to the start (initial position of the target block) and check if move was valid
        if not self.move_arm(self, start_target):
            return 0

        # Set the digital output 0 (suction gripper) to high
        self.set_io(0, 1.0)

        # Apply delay to assure suction gripper has the block attached
        time.sleep(1.0)

        if self.analog_in_0_value > 2.0:
            if not self.move_arm(self, T[start_tower]):
                return 0
        else:
            return 0

        if not self.move_arm(self, T[end_tower]):
            return 0

        if not self.move_arm(self, end_target):
            return 0

        self.set_io(0, 0.0)

        return 1

        ############### Your Code End Here ###############


def main(args=None):
    input("Check if the UR3e is in 'Remote' Mode?\n\
    Check if the UR3e is initialized and in 'Normal' state.\n\
    Have you run the ROS2 launch statement?\n\
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.\n\
    \n\
    Press <Enter> to Continue.")
    rclpy.init(args=args)
    node = UR3e()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input
    loop_count = 0
    start_location = 0
    dest_location = 0
    # Wait for initial state updates
    while node.current_joint_state is None:
        executor.spin_once(timeout_sec=0.05)
        node.get_logger().info("Waiting for initial state updates...")
        time.sleep(0.5)

    try:
        # Get user input
        start_location, dest_location = input("Enter the tower you will start at and want to move the blocks to: ").split(" ")
        print("You want to move from " + start_location + " to " + dest_location + "\n")

        start_location = int(start_location)
        dest_location = int(dest_location)

        if start_location > 2 or dest_location > 2 or start_location < 0 or dest_location < 0:
            print("Enter values in the correct range")
            loop_count = 0
        elif start_location == dest_location:
            print("Start Location and Destination must differ.")
            loop_count = 0
        else:
            loop_count = 1


        ############## Your Code Start Here ##############
        # TODO: modify the code so that UR3e can move tower accordingly from user input

        while(loop_count > 0):
            aux = 0
            if start_location == 0:
                if dest_location == 1:
                    aux = 2
                else:
                    aux = 1
            elif start_location == 1:
                if dest_location == 0:
                    aux = 2
                else:
                    aux = 0
            else:
                if dest_location == 0:
                    aux = 1
                else:
                    aux = 0

            node.move_arm(home)

            node.get_logger().info(f'Sending goal 1 ...')

            if not node.move_arm(Q[0][0]):
                node.get_logger().error("Failed to move to goal" + str(Q[0][0]))
                break

            node.set_io(0, 1.0)  # Turn/ on suction
            # Delay to make sure suction cup has grasped the block
            time.sleep(1.0)

            node.get_logger().info(f'Sending goal 2 ...')
            if not node.move_arm(Q[1][1]):
                node.get_logger().error("Failed to move to goal"+str(Q[1][1]))
                break

            node.get_logger().info(f'Sending goal 3 ...')
            if not node.move_arm(Q[2][2]):
                node.get_logger().error("Failed to move to goal"+str(Q[2][2]))
                break
            loop_count = loop_count - 1
            node.set_io(0, 0.0)  # Turn off suction

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
