from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from time import sleep

# Initialize and instance of the robot class
robot = InterbotixManipulatorXS("px100", "arm", "gripper")

theta = 0.0

while True:

    # Initial position for the robot
    robot.arm.go_to_home_pose()
    robot.gripper.release()

    # Find the correct rotation of the base
    theta_values = []

    with open("theta.txt", "r") as file:
        for line in file:
            # Split the line into values and convert them to floats
            values = line.split()
            for value in values:
                try:
                    theta_values.append(float(value))
                except ValueError:
                    pass  # Ignore non-float values
    
    if len(theta_values) >= 15:
        avg_theta = abs(float((sum(theta_values[-15:]) / 15)))

    else:
        avg_theta = 0.0

    # Find the correct height of the end effector
    z_values = []

    with open("z_robot.txt", "r") as file:
        for line in file:
            # Split the line into values and convert them to floats
            values = line.split()
            for value in values:
                try:
                    z_values.append(float(value))
                except ValueError:
                    pass  # Ignore non-float values
    
    if len(z_values) >= 15:
        avg_z = float((sum(z_values[-15:]) / 15))

    else:
        avg_z = None

    # Find the correct distance of the end effector
    x_values = []

    with open("x_robot.txt", "r") as file:
        for line in file:
            # Split the line into values and convert them to floats
            values = line.split()
            for value in values:
                try:
                    x_values.append(float(value))
                except ValueError:
                    pass  # Ignore non-float values
    
    if len(x_values) >= 15:
        avg_x = float((sum(x_values[-15:]) / 15))

    else:
        avg_x = None

    # Direct robot to follow pen
    robot.arm.set_single_joint_position("waist", avg_theta)
    robot.arm.set_ee_cartesian_trajectory(z = avg_z)
    robot.arm.set_ee_cartesian_trajectory(x = avg_x)
    robot.gripper.grasp()
    robot.arm.go_to_home_pose()
    sleep(1)
    robot.gripper.release()
