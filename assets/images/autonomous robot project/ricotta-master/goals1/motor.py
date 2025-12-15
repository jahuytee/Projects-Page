# Imports
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
        io.set_PWM_dutycycle(self.pinA, 0)
        io.set_PWM_dutycycle(self.pinB, 0)


    # stop method
    def stop(self):
        """
        Stopping motors by setting PWM to 0
        """
        io.set_PWM_dutycycle(self.pinA, 0)
        io.set_PWM_dutycycle(self.pinB, 0)


    # setLevel method
    def setLevel(self, level):
        """
        Sets motor's direction and speed.

        Parameters:
        - level (float): Value of motor power level between -1.0 and 1.0
        Positive value spins the motor forward, and negative reverses it.

        Converts the level to a PWM duty cycle and sets the appropriate pin.
        """

        pwm_value = int(abs(level)*250)

        if level > 0:
            io.set_PWM_dutycycle(self.pinB, pwm_value)
            io.set_PWM_dutycycle(self.pinA, 0)
        elif level < 0:
            io.set_PWM_dutycycle(self.pinA, pwm_value)
            io.set_PWM_dutycycle(self.pinB, 0)
        else:
            self.stop(self)


# GPIO pin assignments for motor control
PIN_MOTOR1_LEGA = 8 # left motor, leg A
PIN_MOTOR1_LEGB = 7 # left motor, leg B
PIN_MOTOR2_LEGA = 6 # right motor, leg A
PIN_MOTOR2_LEGB = 5 # right motor, Leg B


if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Could not connect to pigpio daemon.")
        sys.exit(0)

    motor_left = Motor(io, PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB)
    motor_right = Motor(io, PIN_MOTOR2_LEGA, PIN_MOTOR2_LEGB)


    n = 4
    turn_direction = "right" 
    drive_speed = 0.90
    turn_speed = 0.80
    turn_time = 0.57
    drive_time = 2.8   #time to go one side


    try:
        # Repetition of driving and turning n times
        for i in range(n):
            # Foward drive at specifed speed and time
            motor_left.setLevel(drive_speed)
            motor_right.setLevel(drive_speed)
            time.sleep(drive_time)
            print("going straight")

            motor_left.stop()
            motor_right.stop()
            print("stopping")
            time.sleep(2)
            
            # Rotate in place to the right by setting left 
            # motor forward and right backwards
            if turn_direction == "right":
                motor_left.setLevel(turn_speed)
                motor_right.setLevel(-turn_speed)
                print("turning right")

            # Rotate in place to the right by setting left 
            # motor forward and right backwards
            elif turn_direction == "left":
                motor_left.setLevel(-turn_speed)
                motor_right.setLevel(turn_speed)

            time.sleep(turn_time)
    
    except KeyboardInterrupt:
        print("Robot Force Stop")
    
        
    finally:
        print("final stop")
        motor_left.stop()
        motor_right.stop()

        # Clear all pins, just in case.
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 0)
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 0)
        io.set_PWM_dutycycle(PIN_MOTOR2_LEGA, 0)
        io.set_PWM_dutycycle(PIN_MOTOR2_LEGB, 0)    

