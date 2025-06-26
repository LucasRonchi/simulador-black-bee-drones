from math import cos, sin, pi
import time


# Posição do motores:
# 3           2
#   \       /
#    \     /
#     \___/
#     /   \
#    /     \
#   /       \
# 4           1
# 


class Drone:
    def __init__(self):
        # Drone information
        self.acceleration_per_watt = 60            # m/s^2/watts

        # Drone state
        self.position = [0.0, 0.0, 0.0]            # [x, y, z] in meters
        self.direction = 0.0                       # radians (starting on x-axis and conter-clockwise)
        self.angle_of_inclination = [0.0, 0.0]     # [x, y] in radians (angle of inclination of the drone)
        self.motors_power = [0.0, 0.0, 0.0, 0.0]   # [motor1, motor2, motor3, motor4] in watts

        # Joystick input
        self.joystick_input = [0.0, 0.0, 0.0, 0.0] # [vertical, Rotate, Forward, Lateral] in decimal values

        # Limits
        self.max_horizontal_speed = 0.5              # m/s
        self.max_vertical_speed = 0.5                # m/s
        self.max_angular_speed = 0.5                 # rad/s

        # Time tracking
        self.last_update_time = time.time()


    def update_joystick_input(self, vertical, rotate, forward, lateral):
        self.update()  # Call update to process the joystick input

        # Update joystick input values
        self.joystick_input[0] = vertical          # Vertical movement
        self.joystick_input[1] = rotate            # Rotation
        self.joystick_input[2] = forward           # Forward movement
        self.joystick_input[3] = lateral           # Lateral movement


    def update(self):
        # Calculate delta time
        delta_time = self.calc_delta_time()

        # Calculate speeds based on joystick input
        speed_x, speed_y, speed_z, angular_speed = self.calc_speeds()

        # Update position based on speeds and delta time
        self.update_position(speed_x, speed_y, speed_z, angular_speed, delta_time)

        # Update motors power based on speeds
        self.update_motors()

        # Calculate angle of inclination based on speeds
        self.calc_angle_of_inclination()


    def calc_delta_time(self):
        # Calculate delta time in seconds
        time_now = time.time()
        delta_time = (time_now - self.last_update_time)
        self.last_update_time = time_now

        return delta_time


    def calc_speeds(self):
        # Horizontal (forward)
        speed_x = self.joystick_input[2] * cos(self.direction)
        speed_y = self.joystick_input[2] * sin(self.direction)

        # Horizontal (lateral)
        speed_x += self.joystick_input[3] * cos(self.direction - pi/2)
        speed_y += self.joystick_input[3] * sin(self.direction - pi/2)

        # Vertical
        speed_z = self.joystick_input[0]

        # Rotate
        angular_speed = -self.joystick_input[1]

        # limit horizontal speed
        horizontal_speed = (speed_x**2 + speed_y**2)**0.5
        if horizontal_speed > 1:
            speed_x /= horizontal_speed
            speed_y /= horizontal_speed

        return speed_x, speed_y, speed_z, angular_speed


    def update_position(self, speed_x, speed_y, speed_z, angular_speed, delta_time):
        if self.position[2] <= 0 and speed_z <= 0:
            return

        self.position[0] += speed_x * self.max_horizontal_speed * delta_time
        self.position[1] += speed_y * self.max_horizontal_speed * delta_time
        self.position[2] += speed_z * self.max_vertical_speed * delta_time
        self.direction += angular_speed * self.max_angular_speed * delta_time
        
        if self.position[2] <= 0:
            self.position[2] = 0


    def update_motors(self):
        if self.position[2] == 0:
            # If the drone is not powered on, set all motors to 0
            self.motors_power = [0.0, 0.0, 0.0, 0.0]
            return

        gravity = 10  # m/s^2
        correction_factor = 0.1
        
        for i in range(4):
            # Hover
            power = gravity * self.acceleration_per_watt / 4
            
            # Vertical adjustment
            power *= 1 + (correction_factor * self.joystick_input[0])

            # Horizontal adjustment
            if i == 0:
                power *= 1 + (correction_factor * self.joystick_input[2]) - (correction_factor * self.joystick_input[3])
            elif i == 1:
                power *= 1 - (correction_factor * self.joystick_input[2]) - (correction_factor * self.joystick_input[3])
            elif i == 2:
                power *= 1 - (correction_factor * self.joystick_input[2]) + (correction_factor * self.joystick_input[3])
            elif i == 3:
                power *= 1 + (correction_factor * self.joystick_input[2]) + (correction_factor * self.joystick_input[3])

            # Angular adjustment
            if i == 0 or i == 2:
                power *= 1 - (correction_factor * self.joystick_input[1])
            elif i == 1 or i == 3:
                power *= 1 + (correction_factor * self.joystick_input[1])

            self.motors_power[i] = power


    def calc_angle_of_inclination(self):
        if self.position[2] == 0:
            # If the drone is not powered on, set angle of inclination to 0
            self.angle_of_inclination = [0.0, 0.0]
            return

        correction_factor = 20

        self.angle_of_inclination[0] = correction_factor * self.joystick_input[2]
        self.angle_of_inclination[1] = correction_factor * self.joystick_input[3]


    def get_states(self):
        return {
            'position': self.position,
            'direction': self.direction,
            'angle_of_inclination': self.angle_of_inclination,
            'motors_power': self.motors_power,
            'joystick_input': self.joystick_input
        }
