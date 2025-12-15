import pigpio
from DriveSystem import DriveSystem
from Sense import LineSensor
from street_behaviors import LineFollower, PullForward, TurningBehavior

if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Could not connect to pigpio daemon.")
        exit()

    try:
        left_pins = (8, 7)
        right_pins = (6, 5)
        PIN_IR_LEFT   = 14
        PIN_IR_MIDDLE = 15
        PIN_IR_RIGHT  = 18

        drive = DriveSystem(io, left_pins, right_pins)
        sensor = LineSensor(io, PIN_IR_LEFT, PIN_IR_MIDDLE, PIN_IR_RIGHT)
        follower = LineFollower(io, drive, sensor)

        print("Simple Brain started.")
        
        while True:
            choice = input("Drive Straight, left, or right")

            if choice == "left" or choice == "right":
                duration = TurningBehavior(drive, sensor, choice)
                duration.turn()                

            elif choice == "straight":
                print("Following line")
                result = follower.follow_line()

                if result == "intersection":
                    print("Intersection reached.")
                    forward = PullForward(drive, duration=0.5)
                    forward.pull_forward()

                elif result == "end":
                    print("Reached end of the street")
                    follower.turning_behavior(left)
                    follower.follow_line()

                else:
                    print("Unknown line-following result:", result)

            else:
                print("Invalid input. Please enter 'straight', 'left', or 'right'.")

    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    finally:
        drive.stop()
        io.stop()
        print("Robot stopped. Brain shutdown :3")
