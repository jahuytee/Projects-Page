import pigpio


class IR:
    """
    A low-level class to interface with a single digital IR sensor.
    Uses pigpio to read high/low values from a GPIO pin.
    """
    def __init__(self, io, pin):
        """
        Initialize a single IR sensor.
        Parameters:
        - io (pigpio.pi): The pigpio interface for controlling GPIO pins
        - pin (int): GPIO pin number the sensor is connected to
        """
        self.io = io
        self.pin = pin
        self.io.set_mode(pin, pigpio.INPUT)

    def read(self):
        """
        Returns 1 if black tape is detected, and
        0 if white floor is detected
        """
        return self.io.read(self.pin)


class LineSensor:
    """
    A mid-level class that combines three IR sensors into one logical unit.
    It reads left, middle, and right IR sensors and returns a 3-value tuple.
    """
    def __init__(self, io, pin_left, pin_middle, pin_right):
        """
        Initialize LineSensor with three IR sensors.

        Parameters:
        - io (pigpio.pi): pigpio interface object
        - pin_left (int): GPIO pin for the left IR sensor
        - pin_middle (int): GPIO pin for the middle IR sensor
        - pin_right (int): GPIO pin for the right IR sensor
        """

        self.left = IR(io, pin_left)
        self.middle = IR(io, pin_middle)
        self.right  = IR(io, pin_right)

    def read(self):
        """
        Returns a tuple (L, M, R) of 0s and 1s
        """
        L = self.left.read()
        M = self.middle.read()
        R = self.right.read()
        return (L, M, R)
