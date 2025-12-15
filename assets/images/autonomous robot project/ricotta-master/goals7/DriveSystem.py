import pigpio
import sys
import time
import traceback


class Motor:
    """
    Motor class to control bidirectional motors via PWM using pigpio.
    """
    def __init__(self, io, pinA, pinB):
        """
        Initializes motor instance

        Parameters:
        - io (pigpio.pi): The pigpio interface for controlling GPIO pins
        - pinA (int): GPIO pin number for one motor input
        - pinB (int): GPIO pin number for the other motor input

        Sets both pins to output mode, configures their PWM frequency and range,
        and initializes with a duty cycle of 0
        """
        self.io = io
        self.pinA = pinA
        self.pinB = pinB

        # Configure pinA and pinB as outputs using pigpio
        io.set_mode(self.pinA, pigpio.OUTPUT)
        io.set_mode(self.pinB, pigpio.OUTPUT)

        # Set pwm range and frequency
        io.set_PWM_range(self.pinA, 255)
        io.set_PWM_range(self.pinB, 255)

        io.set_PWM_frequency(self.pinA, 1000)
        io.set_PWM_frequency(self.pinB, 1000)

        #set duty cycle to 0
        self.stop()


    # stop method
    def stop(self):
        """ 
        Stopping motors by setting PWM to 0
        """
        self.setLevel(0)


    # setLevel method
    def setLevel(self, level):
        """
        Sets motor's direction and speed.

        Parameters:
        - level (float): Value of motor power level between -1.0 and 1.0
        Positive value spins the motor forward, and negative reverses it.

        Converts the level to a PWM duty cycle and sets the appropriate pin.
        """

        pwm_value = int(abs(level) * 250)

        if level >= 0:
            self.io.set_PWM_dutycycle(self.pinB, pwm_value)
            self.io.set_PWM_dutycycle(self.pinA, 0)
        elif level < 0:
            self.io.set_PWM_dutycycle(self.pinA, pwm_value)
            self.io.set_PWM_dutycycle(self.pinB, 0)
        else:
            self.io.set_PWM_dutycycle(self.pinA, 0)
            self.io.set_PWM_dutycycle(self.pinB, 0)



class DriveSystem:
    def __init__(self, io):
        """
        Initialize the DriveSystem with two Motor instances.
        """
        left_pins = (8, 7)
        right_pins = (6, 5)

        self.motor_left = Motor(io, *left_pins)
        self.motor_right = Motor(io,*right_pins)

        drive_power = 0.83

        self.modes = {"straight" : (drive_power + 0.02, drive_power), 
                      "veer_r" : (0.84, 0.69), 
                      "steer_r" : (0.90,0.65), 
                      "turn_r" : (0.90,0.55), 
                      "hook_r" : (0.77,0), 
                      "spin_r" : (0.79,-0.79), 
                      "veer_l" : (0.71, 0.80), 
                      "steer_l" : (0.70, 0.86), 
                      "turn_l" : (0.60, 0.92), 
                      "hook_l" : (0, 0.77),
                      "spin_l" : (-0.79, 0.79)}

    def stop(self):
        """
        Stops both motors.
        """
        self.motor_left.stop()
        self.motor_right.stop()

    def drive(self, mode, reverse=False):
        """
        Looks for the mode selected as a parameter in a pre-constructed dictionary of turn modes
        """
        if mode in self.modes:
            left_level, right_level = self.modes[mode]
            if reverse:
                left_level = -left_level
                right_level = -right_level
            self.motor_left.setLevel(left_level)
            self.motor_right.setLevel(right_level)
        else:
            print("This is not a valid drive mode")   
    
    def pwm(self, PWM_L, PWM_R):
        """
        Directly set the left and right motor PWM values.
        PWM_L, PWM_R: float values between -1.0 and 1.0
        """
        self.motor_left.setLevel(PWM_L)
        self.motor_right.setLevel(PWM_R)








       


