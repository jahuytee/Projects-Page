import time
import traceback
import pigpio
from DriveSystem import DriveSystem
from Sense import LineSensor
from street_behaviors import Behaviors
from AngleSensor import AngleSensor

if __name__ == "__main__":
    try:
        io = pigpio.pi()
        if not io.connected:
            print("Could not connect to pigpio daemon.")
            exit()
            
        anglesensor = AngleSensor(io)
        drive = DriveSystem(io)
        values = []
        t0 = time.time()

        while True:
            t_now = time.time()
            drive.drive("spin_l")
            values.append(anglesensor.readadc(1))
            if t_now >= t0 + 12:
                break

        print(f"max of values is {max(values)}, min of values is {min(values)}")    

    finally:
        drive.stop()
        io.stop()
		

