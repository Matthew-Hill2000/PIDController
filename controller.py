import numpy as np
import pygame

wind_active = True  # Select whether you want to activate wind or not


# Helper Functions
def clamp(value, min_value, max_value):
    """Ensures that a value stays within a specified range."""
    return max(min(value, max_value), min_value)

# PID Controller Class
class PIDController:
    def __init__(self, kp, ki, kd, min_output=None, max_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_error = 0
        self.previous_error = 0
        self.min_output = min_output
        self.max_output = max_output

    def compute(self, error, dt):
        # Update integral errors
        self.integral_error += error * dt
        # Compute derivative terms
        derivative = (error - self.previous_error) / dt
        # PID outputs for both axes
        output = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative)

        # Store the current error for the next iteration
        self.previous_error = error

        # Clamp output between min_output and max_output if defined
        if output > self.max_output and self.max_output is not None:
            output = self.max_output
        elif output < self.min_output and self.min_output is not None:
            output = self.min_output

        return output


# Main Cascade Controller Function (Advanced Method)
def controller(state, target_pos, dt):
    """
    Controller function to compute motor thrusts based on the current state and target position.

    Args:
        state: A list or array containing the current state information:
            [x_position, y_position, x_velocity, y_velocity, attitude, angular_velocity].
        target_pos: A numpy array containing the target [x, y] position.
        dt: Time step for computations.

    Returns:
        Thrust values for the left (u1) and right (u2) motors.
    """
    # Define maximum and minimum thrust for each thruster
    thrust_minimum = 0.2
    thrust_maximum = 0.8

    # Define PID Controllers for each axis
    x_pos_PID = PIDController(1.2, 0.16, 0.01, -1, 1)
    x_speed_PID = PIDController(0.2, 0.1, 0.0065, -1, 1) 
    attitude_PID = PIDController(0.56, 0.16, 0.04, -1, 1)
    attitude_speed_PID = PIDController(8.0, 1.0, 0.2, -1.0, 1.0)
    y_pos_PID = PIDController(10.0, 5.0, 0.0, -1, 1)
    y_speed_PID = PIDController(10.0, 5.0, 0.0, -1, 1)


    # Unpack state array
    x_pos, x_vel = state[0], state[2]
    y_pos, y_vel = state[1], state[3]
    attitude, angular_vel = state[4], state[5]
    
    # X-axis Control
    x_error = x_pos - target_pos[0]
    optimal_x_speed = -x_pos_PID.compute(x_error, dt)
    x_speed_error = x_vel - optimal_x_speed 
    desired_rot = - x_speed_PID.compute(x_speed_error, dt) # -np.pi/2

    
    # Clamp desired rotation within bounds
    rot_min =  - (np.pi / 4) # -np.pi/2
    rot_max =  + (np.pi / 4) # -np.pi/2
    desired_rot = min(max(desired_rot, rot_min), rot_max)


    # Rotation control
    rot_error = attitude - desired_rot
    optimal_angular_speed = -attitude_PID.compute(rot_error, dt)
    angular_speed_error = optimal_angular_speed - angular_vel
    desired_angular_speed = attitude_speed_PID.compute(angular_speed_error, dt)
    left_thrust = desired_angular_speed
    right_thrust = -left_thrust

    # Y-axis Control
    y_error = y_pos - target_pos[1]
    print(y_error)
    desired_yvel = y_pos_PID.compute(-y_error, dt)
    print(desired_yvel)
    yvel_error = y_vel - desired_yvel
    print(yvel_error)
    base_thrust = y_speed_PID.compute(yvel_error, dt)
    print(base_thrust)
    
    # Clamp base thrust within bounds
    base_thrust = min(max(base_thrust, thrust_minimum), thrust_maximum)
    print(base_thrust)
    
    #Combine base thrust with the thrust needed to cause a rotation
    thruster_left = base_thrust + left_thrust
    thruster_right = base_thrust + right_thrust

    return thruster_left, thruster_right 

if __name__ == "__main__":
    #Print returned thrust values for specific defined state for analysis purposes
    print(controller([1, 3, -0.2, 0, -0.2, 0], [1, 3], 0.1))