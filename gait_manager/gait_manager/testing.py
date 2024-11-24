import os
import pandas as pd

actions_index = 0
desired_positions = {}

WORKSPACE_NAME = "WORMS-software-ws"
REPO_NAME = "WORMS-gait-manager"
PACKAGE_NAME = "gait_manager"

working_file_path = os.path.dirname(os.path.realpath(__file__))
end_index = working_file_path.find(WORKSPACE_NAME) + len(WORKSPACE_NAME)
script_directory = os.path.join(working_file_path[:end_index], "src", REPO_NAME, PACKAGE_NAME, PACKAGE_NAME)


def get_waypoints(action, script_directory):
    """
    Returns waypoints from csv file given some action.
    """
    waypoints_path = os.path.join(script_directory, "gait_data", f"{action}.csv")

    df = pd.read_csv(waypoints_path)
    return [list(i) for i in df.values]

def get_actions():
    """
    Returns waypoints from csv file given some action.
    """

    WORKSPACE_NAME = "WORMS-software-ws"
    REPO_NAME = "WORMS-gait-manager"
    PACKAGE_NAME = "gait_manager"

    working_file_path = os.path.dirname(os.path.realpath(__file__))
    end_index = working_file_path.find(WORKSPACE_NAME) + len(WORKSPACE_NAME)
    script_directory = os.path.join(working_file_path[:end_index], "src", REPO_NAME, PACKAGE_NAME, PACKAGE_NAME)

    action = "run_stand_forward_gait"

    waypoints_path = os.path.join(script_directory, f"{action}.csv")

    df = pd.read_csv(waypoints_path)

    return df

def interpolate_waypoints(waypoints, increment=0.1):
        """
        Transition from the current position to each array in sequence by 0.1 increments, producing a comprehensive list of
        lists representing each incremental step towards the waypoints.

        :param arrays: A sequence of arrays where each array is a list of numbers.
        :param increment: The incremental value to adjust the numbers. Default is 0.1.
        :return: A list of lists representing each step through the waypoints.
        """
        transition_steps = []

        # Start from the current position
        current_state = [0,0,0]
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

all_actions_df = get_actions()

waypoints = get_waypoints(all_actions_df["Swan"][1], script_directory)
print(interpolate_waypoints(waypoints))