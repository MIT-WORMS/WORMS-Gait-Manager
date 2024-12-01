import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.context import Context
import numpy as np
import pandas as pd
import subprocess
import platform
import os
import time
import sys
import signal
from array import array

from messages.msg import Personal, Configuration, State, System, Readiness, Command # type: ignore

class Gait_Manager(Node):
    def __init__(self):
        super().__init__("gait_manager")

        self.worm_id = self.get_namespace()[1:]

        # just for testing
        self.state_dict = {}
        init = JointState()
        init.position = [1.0 for _ in range(3)]
        init.velocity = [0.0 for _ in range(3)]
        init.effort = [0.0 for _ in range(3)]
        self.state_dict[self.worm_id] = init

        WORKSPACE_NAME = "WORMS-software-ws"
        REPO_NAME = "WORMS-gait-manager"
        PACKAGE_NAME = "gait_manager"

        working_file_path = os.path.dirname(os.path.realpath(__file__))
        end_index = working_file_path.find(WORKSPACE_NAME) + len(WORKSPACE_NAME)
        self.script_directory = os.path.join(working_file_path[:end_index], "src", REPO_NAME, PACKAGE_NAME, PACKAGE_NAME)

        self.state_subscriber = self.create_subscription(State, "state_topic", self.state_callback, 10)
        self.config_subscriber = self.create_subscription(Configuration, "config_topic", self.config_callback, 10)
        self.cmd_subscriber = self.create_subscription(Command, "cmd_topic", self.cmd_callback, 10)

        self.joint_cmd_publisher = self.create_publisher(JointState, 'joint_cmd_topic', 10)

        self.testing_publisher = self.create_publisher(JointState, 'joint_states_topic', 10)

        # self.state_dict = {}

        # whether or not the positions of worms align with where they should be
        self.state_readiness = False
        self.start_of_command = True
        # whether or not worm is "Active"
        self.config_readiness = False

        # dict of worm:position
        self.desired_positions = {}
        self.all_actions_df = {}
        self.all_actions_len = 0
        self.actions_index = 0

        self.config = None

        self.command_action = None
        self.command_duration = 0
        self.command_start = 0

        self.timer = self.create_timer(0.02, self.timer_callback)
        # self.testing_timer = self.create_timer(0.1, self.testing_callback)



    def is_state_valid(self):
        """
        Returns whether or not state_dict is valid. Needs to be changed.
        """
        # might instead want brain to publish a 'configuration name' or smth
        return set(self.state_dict.keys()) == set(["Duck","Swan","Lion","Pony","Goat","Frog"])

    def compute_state_readiness(self):
        """
        Checks to see if state_readiness should be true and adjusts accordingly.
        """
        state_readiness = True
        for i, state in self.state_dict.items():
            if i not in self.desired_positions or self.desired_positions[i] != state.position:
                state_readiness = False
                try:
                    self.get_logger().info(f"Mismatch: {self.desired_positions[i]}, {state.position}")
                except KeyError:
                    pass
        if not self.is_state_valid():
            state_readiness = False
        self.state_readiness = state_readiness

    def state_callback(self, msg):
        for i, worm in enumerate(msg.worms):
            if worm != self.worm_id:
                self.state_dict[worm] = msg.positions[i]
            # if worm == self.worm_id:
            #     self.get_logger().info(f"Reset pos: {self.state_dict[worm].position}")

        self.compute_state_readiness()

            
    def config_callback(self, msg):
        self.config_readiness = msg.is_active
        self.config = msg.personal.location

    def cmd_callback(self, msg):
        if self.command_action != msg.action:
            self.command_action = msg.action
            self.command_start = time.time()
            self.start_of_coomand = True
            self.actions_index = 0
        self.command_duration = msg.duration

    def get_actions(self):
        """
        Returns waypoints from csv file given some action.
        """
        if self.command_action == "forward":
            action = "run_stand_forward_gait"
        waypoints_path = os.path.join(self.script_directory, f"{action}.csv")

        df = pd.read_csv(waypoints_path)

        return df
    
    def get_waypoints(self, action):
        """
        Returns waypoints from csv file given some action.
        """
        waypoints_path = os.path.join(self.script_directory, "gait_data", f"{action}.csv")

        df = pd.read_csv(waypoints_path)
        return [list(i) for i in df.values]
    
    def publish_to_controller(self, waypoints):
        """
        Interpolates waypoints and publishes to controller.
        """
        for interpolated_waypoint in self.interpolate_waypoints(waypoints):
            msg = JointState()
            msg.position = interpolated_waypoint
            msg.velocity = [0 for _ in range(len(interpolated_waypoint))]
            msg.effort = [0 for _ in range(len(interpolated_waypoint))]

            self.joint_cmd_publisher.publish(msg)

        # self.state_dict[self.worm_id] = msg
        self.testing_publisher.publish(msg)
        self.get_logger().info(f"Successfully changed dict: {msg.position}")

    def testing_callback(self):
        self.testing_publisher.publish(self.state_dict[self.worm_id])

    def interpolate_waypoints(self, waypoints, increment=0.1):
        """
        Transition from the current position to each array in sequence by 0.1 increments, producing a comprehensive list of
        lists representing each incremental step towards the waypoints.

        :param arrays: A sequence of arrays where each array is a list of numbers.
        :param increment: The incremental value to adjust the numbers. Default is 0.1.
        :return: A list of lists representing each step through the waypoints.
        """
        transition_steps = []

        # Start from the current position
        current_state = self.state_dict[self.worm_id].position

        for waypoint in waypoints:
            while True:
                step = []
                done = True
                for current_val, target_val in zip(current_state, waypoint):
                    if abs(target_val - current_val) > increment:
                        done = False
                        if target_val > current_val:
                            step.append(current_val + increment)
                        else:
                            step.append(current_val - increment)
                    else:
                        step.append(target_val)
                
                current_state = step
                transition_steps.append(step)
                
                if done:
                    break

        return transition_steps

    def timer_callback(self):

        self.compute_state_readiness()
        # self.get_logger().info(f"{self.config_readiness}, {self.start_of_command}, {self.is_state_valid()}")
        # self.get_logger().info(f"{self.state_dict.keys()}")
        self.get_logger().info(f"{self.state_readiness}, {self.is_state_valid()}, {self.command_duration > time.time()-self.command_start}")
        # self.get_logger().info(f"{self.state_dict[self.worm_id].position}, {self.is_state_valid()}")
        if self.config_readiness and (self.state_readiness or (self.start_of_command and self.is_state_valid())) and self.command_duration > time.time()-self.command_start:

            if self.start_of_command:
                self.all_actions_df = self.get_actions()
                self.all_actions_len = self.all_actions_df.shape[0]
                self.start_of_command = False
            
            # set desired positions
            for i in self.all_actions_df:
                waypoints = self.get_waypoints(self.all_actions_df[i][self.actions_index])
                self.desired_positions[i] = array('d', waypoints[-1])
                if i == self.worm_id:
                    self.publish_to_controller(waypoints)
                    self.get_logger().info(f"PUBLISH PUBLISH: {waypoints[-1]}")

            if self.actions_index+1 == self.all_actions_len:
                self.actions_index = 0
            else:
                self.actions_index += 1

def shutdown_nodes(signal, frame):
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    """
    Initializes ROS, creates 'gait_manager' node.
    """
    rclpy.init(args=args)

    node = Gait_Manager()

    signal.signal(signal.SIGINT, shutdown_nodes)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
            

            


        