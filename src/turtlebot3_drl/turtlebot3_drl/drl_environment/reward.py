# Modified by: Seunghyeop
# Description: This code has been modified to train the Turtlebot3 Waffle_pi model.

from ..common.settings import REWARD_FUNCTION, COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE, SUCCESS, TIMEOUT, RESULTS_NUM, THRESHOLD_COLLISION, STEP_TIME

goal_dist_initial = 0

reward_function_internal = None

def get_reward(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance):
    return reward_function_internal(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance)

#def get_reward(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance, min_lidar_dist):
#    return reward_function_internal(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance, min_lidar_dist)

#def get_reward(succeed, action_linear, action_angular, distance_to_goal, distance_to_goal_privious, goal_angle, min_obstacle_distance,  normalized_laser):
#    return reward_function_internal(succeed, action_linear, action_angular, distance_to_goal, distance_to_goal_privious, goal_angle, min_obstacle_distance,  normalized_laser)

def get_reward_A(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist):
        # [-3.14, 0]
        r_yaw = -1 * abs(goal_angle)

        # [-4, 0]
        r_vangular = -1 * (action_angular**2)

        # [-1, 1]
        r_distance = (2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1

        # [-20, 0]
        if min_obstacle_dist < 1.4 * THRESHOLD_COLLISION:
            r_obstacle = -20
        else:
            r_obstacle = 0

        # [-2 * (2.2^2), 0]
        r_vlinear = -1 * (((0.26 - action_linear) * 10) ** 2)

        reward = r_yaw + r_distance + r_obstacle + r_vlinear + r_vangular - 1

        if succeed == SUCCESS:
            reward += 2500
        elif succeed == COLLISION_OBSTACLE or succeed == COLLISION_WALL:
            reward -= 2000
        return float(reward)


def get_reward_B(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist):
        # [-3.14, 0]
        r_yaw = -1 * abs(goal_angle)

        # [-4, 0]
        r_vangular = -1 * (action_angular**2)

        # [-10, 10]
        r_distance = ((2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1) * 10

        # [-50, 0]
        if min_obstacle_dist < 1.4 * THRESHOLD_COLLISION:
            r_obstacle = -50
        else:
            r_obstacle = 0

        # [-2 * (2.6^2), 0]
        r_vlinear = -1 * (((0.26 - action_linear) * 10) ** 2)

        reward = r_yaw + r_distance + r_obstacle + r_vlinear + r_vangular - 1

        if succeed == SUCCESS:
            reward += 5000
        elif succeed == COLLISION_OBSTACLE or succeed == COLLISION_WALL:
            reward -= 2000
        return float(reward)
        
def get_reward_C(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist, min_lidar_dist):
        # [-3.14, 0]
        r_yaw = -1 * abs(goal_angle)

        # [-4, 0]
        r_vangular = -1 * (action_angular**2)

        # [-1, 1]
        r_distance = (2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1

        # [-80, 0]
        if min_obstacle_dist < 1.4 * THRESHOLD_COLLISION:
            r_obstacle = -80
        else:
            r_obstacle = 0

         # [-20, 0]
        if min_lidar_dist < 1.4 * THRESHOLD_COLLISION:
            r_wall = -20
        else:
            r_wall = 0

        # [-2 * (2.2^2), 0]
        r_vlinear = -1 * (((0.26 - action_linear) * 10) ** 2)

        reward = r_yaw + r_distance + r_obstacle + r_vlinear + r_vangular + r_wall - 1

        if succeed == SUCCESS:
            reward += 5000
        elif succeed == COLLISION_OBSTACLE or succeed == COLLISION_WALL:
            reward -= 2000
        return float(reward)


def get_reward_D(succeed, action_linear, action_angular, goal_dist, goal_dist_previous, goal_angle, min_obstacle_dist, normalized_laser):
    # distance reward
    time_step = STEP_TIME
    distance_reward = goal_dist - goal_dist_previous
    
    # angular reward
    r_yaw = -1 * abs(goal_angle)
    
    angular_punish_reward = 0
    if action_angular > 0.8:
        angular_punish_reward = -1
    if action_angular < -0.8:
        angular_punish_reward = -1
    angular_punish_reward += r_yaw
    
    # linear reward
    linear_punish_reward = 0
    if action_linear < 0.2:
        linear_punish_reward = -2
    
    # collision, success reward
    if min_obstacle_dist < 2 * THRESHOLD_COLLISION:
        collision_reward = -80
    else:
        collision_reward = 0
        
    laser_reward = sum(normalized_laser)-24
    collision_reward += laser_reward
    
    arrive_reward = 0
    
    if succeed == SUCCESS:
        arrive_reward += 100
    elif succeed == COLLISION_OBSTACLE or succeed == COLLISION_WALL:
        collision_reward -= 200

    
    reward  = distance_reward*(5/time_step)*1.2*7 + arrive_reward + collision_reward + angular_punish_reward + linear_punish_reward

    return float(reward)

# Define your own reward function by defining a new function: 'get_reward_X'
# Replace X with your reward function name and configure it in settings.py

def reward_initalize(init_distance_to_goal):
    global goal_dist_initial
    goal_dist_initial = init_distance_to_goal

function_name = "get_reward_" + REWARD_FUNCTION
reward_function_internal = globals()[function_name]
if reward_function_internal == None:
    quit(f"Error: reward function {function_name} does not exist")
