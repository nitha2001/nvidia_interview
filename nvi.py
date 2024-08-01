"""Basic example of using RoboSuite"""

import numpy as np
import robosuite as suite
#import method inverse_kinematics as ik
from robosuite.controllers.ik import InverseKinematicsController
from robosuite.controllers import load_controller_config

# create environment instance
# env = suite.make(
#     env_name="PickPlace", # try with other tasks like "Stack" and "Door"
#     robots="Panda",  # try with other robots like "Sawyer" and "Jaco"
#     has_renderer=True,
#     has_offscreen_renderer=False,
#     use_camera_obs=False,
# )

# # reset the environment
# env.reset()

# for i in range(1000):
#     action = np.random.randn(env.robots[0].dof) # sample random action
#     obs, reward, done, info = env.step(action)  # take action in the environment
#     env.render()  # render on display



# LETS TRY TO DO A PID CONTROL FRO BASIC GIVEN POSITIONS

# Get position of the objects in the environment



# def pickplacepolicy(env):
#     # Get position of the objects in the environment
#     target_pos = env.sim.data.get_site_xpos('goal')[:3]
#     object_pos = env.sim.data.get_site_xpos('object0')[:3]
#     gripper_pos = env.sim.data.get_site_xpos('robot0:grip')[:3]

#     # Compute the error
#     error = target_pos - object_pos
#     print("error: ", error)

#     # Compute the action
#     action = 0.1 * error

#     return action



class Pidcontroller:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = np.zeros(3)
        self.integral = np.zeros(3)


    def compute_action(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.prev_error
        action = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return action
    


Kp = 0.5* np.array([1.0, 1.0, 1.0])
Ki = np.array([0.0, 0.0, 0.0])
Kd = 0.5* np.array([0.1, 0.1, 0.1])

pid_controller = Pidcontroller(Kp, Ki, Kd)


# Lets make an env

# create environment instance
env = suite.make(
    env_name="PickPlace", # try with other tasks like "Stack" and "Door"
    robots="Panda",  # try with other robots like "Sawyer" and "Jaco"
    controller_configs=load_controller_config(default_controller="OSC_POSITION"),
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
)




env.reset()
gripper_closed = False

for obj_name in 'Milk', 'Bread', 'Cereal', 'Can':
    objvisual = 'Visual' + obj_name
    obs = env.reset()
    done = False
    gripper_closed = False
    target_pos = env.sim.data.body_xpos[env.obj_body_id['Milk']]  # Replace with actual target object name
    print("target pos", target_pos)
    target_bin_pos = env.sim.data.body_xpos[env.obj_body_id[obj_name]]  # Replace with actual target bin name
    print("target bin pos", target_bin_pos)
    # Now move the robot to the target position
    while not done:
        # for _ in range(1):
        current_pos = obs['robot0_eef_pos']
        current_ori = obs['robot0_eef_quat']

        if not gripper_closed: #grip to be open
            action = np.zeros(env.action_dim)

            print("initail action", action)
            action[:3] = pid_controller.compute_action(target_pos, current_pos)
            env.step(action)

            # Check if the gripper is close to the object
            if np.linalg.norm(current_pos - target_pos) < 0.08:
                print("Gripper close to object")
                gripper_closed = True
                env.robots[0].grip_action(1)  # Close the gripper

        if gripper_closed:
            # Move to the target bin)
            action = np.zeros(env.action_dim)

            action[:3]= pid_controller.compute_action(target_bin_pos, current_pos)
            obs, _, _, _ = env.step(action)

            # Check if the gripper is close enough to the bin to release the object
            if np.linalg.norm(target_bin_pos - current_pos) < 0.02:
                env.robots[0].grip_action(-1)  # Open the gripper
                done = True

        obs, _, _, _ = env.step(action)
        print("1 loop obs", obs)
        print("type", type(obs))
        env.render()

env.close()



"""
# THINGS I WOULD DO TO GET WORKING if I had more time:
1. THE XPOS could be bottom position so, I would find the height of object, and use it to calculate the target position
2. I would adjust pid paramas to slow the robot down as it reaches the target position
3. I would plan the path to avoid obstacles, this way i could use some kind of joint controle to avoid obstacles by every joint and interpolate to links 
4. or i would create a workspace around the robot and check if I work space is not getting collided with any other obstacles

"""