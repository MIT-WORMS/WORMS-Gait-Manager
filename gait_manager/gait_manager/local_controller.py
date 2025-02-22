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

class LocalController(Node):
    def __init__(self):
        super().__init__('local_controller')

        WORKSPACE_NAME = "WORMS-software-ws"
        REPO_NAME = "WORMS-gait-manager"
        PACKAGE_NAME = "gait_manager"

        working_file_path = os.path.dirname(os.path.realpath(__file__))
        end_index = working_file_path.find(WORKSPACE_NAME) + len(WORKSPACE_NAME)
        self.script_directory = os.path.join(working_file_path[:end_index], "src", REPO_NAME, PACKAGE_NAME, PACKAGE_NAME)

        self.state_subscriber_dict = {}
        # listening to state of worm from brain
        # should the worms be communicating about this to each other now?
            # i dont think so, i think they should all just communicate with local controller
            # if so, dont even need custom msg type, just use jointstate
            # this change is necessary for the code to work because it assumes jointstates
        self.system_state_dict = {}
        self.readiness_dict = {}

        self.readiness_subscriber = self.create_subscription(Readiness, "readiness_communication_topic", self.readiness_callback, 10)
        # should be active

        self.publisher_dict = {}

        self.command_subscriber = self.create_subscription(Command, "local_controller_command_topic", self.lc_command_callback, 10)
        # i think only the leader needs to send this command
        # forwarded from mission control
        self.start_of_command = True
        self.actions_index = 0
        self.desired_positions = {}

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.checking_timer = self.create_timer(2, self.checking_timer_callback)
        self.worm_delay_dict = {}

    # CALLBACK FUNCTIONS
    def readiness_callback(self, msg):
        """
        Intercepts readiness messages from brain communications and tracks
        whether brains are ready. Also used to establish list of all worms.
        """
        if msg.sender not in self.state_subscriber_dict:
            self.state_subscriber_dict[msg.sender] = self.create_subscription(State, f"/{msg.sender}/state_topic", self.generate_state_callback(msg.sender), 10)
            self.publisher_dict[msg.sender] = self.create_publisher(JointState, f"/{msg.sender}/gait_manager_topic", 10)
        self.readiness_dict[msg.sender] = msg.status == "Active"
        self.worm_delay_dict[msg.sender] = time.time()

    def generate_state_callback(self, worm):
        """
        Creates callback functions for each worm
        """
        def state_callback(self, msg):
            self.system_state_dict[worm] = msg
            # this only works if State is replaced with JointState
            # i.e. worms no longer communicating about state
        return state_callback
    
    def lc_command_callback(self, msg):
        """
        Saves command from mission control
        """
        self.command_duration = msg.duration # int64
        self.command_action = msg.action # string
        self.actions_index = 0

    # HELPER FUNCTIONS
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
    
    def publish_to_controller(self, waypoints, worm):
        """
        Interpolates waypoints and publishes to controller.
        """
        for interpolated_waypoint in self.interpolate_waypoints(waypoints, worm):
            msg = JointState()
            msg.position = interpolated_waypoint
            msg.velocity = [0 for _ in range(len(interpolated_waypoint))]
            msg.effort = [0 for _ in range(len(interpolated_waypoint))]

            self.publisher_dict[worm].publish(msg)

    def interpolate_waypoints(self, waypoints, worm, increment=0.1):
        """
        Transition from the current position to each array in sequence by 0.1 increments, producing a comprehensive list of
        lists representing each incremental step towards the waypoints.

        :param arrays: A sequence of arrays where each array is a list of numbers.
        :param increment: The incremental value to adjust the numbers. Default is 0.1.
        :return: A list of lists representing each step through the waypoints.
        """
        transition_steps = []

        # Start from the current position
        current_state = self.system_state_dict[worm].position

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

    def compute_readiness(self):
        """Returns True if all worms ready"""
        for worm in self.readiness_dict:
            if not self.readiness_dict[worm]:
                return False
        return True

    def compute_state_readines(self):
        """
        Computes whether all worms are in the correct positions and ready to move to next action.
        """
        state_readiness = True
        for worm, state in self.system_state_dict.items():
            if worm not in self.desired_positions or self.desired_positions[worm] != state.position:
                state_readiness = False
        return state_readiness

    # TIMER CALLBACKS
    def checking_timer_callback(self):
        """
        Checks that message has been received from each worm in
        a given interval. If not, deletes instances of that worm
        from all dictionaries.
        """
        for worm in self.readiness_dict:
            if time.time() - self.worm_delay_dict[worm] > 2.5:
                all_dicts = [self.state_subscriber_dict,
                             self.system_state_dict,
                             self.readiness_dict,
                             self.publisher_dict,
                             self.worm_delay_dict]
                for dict in all_dicts:
                    try:
                        del dict[worm]
                    except KeyError:
                        pass

    def timer_callback(self):
        """
        Checks whether worms are in correct position and publishes next position.
        """
        if self.compute_readiness() and self.compute_state_readines() and (self.start_of_command or time.time()-self.command_start < self.command_duration):
            if self.start_of_command:
                self.all_actions_df = self.get_actions()
                self.all_actions_len = self.all_actions_df.shape[0]
                self.start_of_command = False

            for worm in self.all_actions_df:
                waypoints = self.get_waypoints(self.all_actions_df[worm][self.actions_index])
                self.desired_positions[worm] = array('d', waypoints[-1])
                self.publish_to_controller(waypoints, worm)

            if self.actions_index+1 == self.all_actions_len:
                self.actions_index = 0
            else:
                self.actions_index += 1

def shutdown_nodes(signal, frame):
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    """
    Initializes ROS, creates 'local_controller' node.
    """
    rclpy.init(args=args)

    node = LocalController()

    signal.signal(signal.SIGINT, shutdown_nodes)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
            


