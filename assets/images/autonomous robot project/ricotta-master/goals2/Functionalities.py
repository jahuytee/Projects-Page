import time
import pigpio
from DriveSystem import DriveSystem
from Sense import LineSensor

# Set your sensor GPIO pins here
PIN_IR_LEFT   = 14
PIN_IR_MIDDLE = 15
PIN_IR_RIGHT  = 18

# Feedback law for line following
FEEDBACK_TABLE = {
    (0, 1, 0): "straight",
    (0, 1, 1): "turn_r",
    (0, 0, 1): "hook_r",
    (1, 1, 0): "turn_l",
    (1, 0, 0): "hook_l",
    (1, 1, 1): "straight",
    #(0,0,0):   spin
}

def flower_power(io):
    print("Starting Flower Power test")

    left_pins = (8, 7)
    right_pins = (6, 5)

    drive = DriveSystem(io, left_pins, right_pins)

    try:
        for mode in drive.modes:
            print(f"\nCurrent mode: {mode}")

            drive.drive(mode)
            time.sleep(4)  
            drive.stop()
            time.sleep(5)

        print("\nDone with Flower Power test.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        drive.stop()
        print("Done with Flower Power test.")


def ir_test(io):
    print("Starting IR sensor test")
    sensor = LineSensor(io, PIN_IR_LEFT, PIN_IR_MIDDLE, PIN_IR_RIGHT)
    try:
        while True:
            L, M, R = sensor.read()
            print(f"IRs: L={L}  M={M}  R={R}")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("IR test interrupted.")


def line_following(io):
    print("Starting line following")
    left_pins = (8, 7)
    right_pins = (6, 5)

    drive = DriveSystem(io, left_pins, right_pins)
    sensor = LineSensor(io, PIN_IR_LEFT, PIN_IR_MIDDLE, PIN_IR_RIGHT)
    
    last_seen_direction = "straight"  # default/fallback direction

    try:
        while True:
            L, M, R = sensor.read()
            state = (L, M, R)

            # Determine action
            if state in FEEDBACK_TABLE:
                action = FEEDBACK_TABLE[state]
                # Update memory if it's a directional state
                if action in ["steer_l", "turn_l"]:
                    last_seen_direction = "left"
                elif action in ["steer_r", "turn_r"]:
                    last_seen_direction = "right"
                elif action == "straight":
                    last_seen_direction = "straight"
            else:
                # Lost line: use last known direction
                if last_seen_direction == "left":
                    action = "spin_l"
                elif last_seen_direction == "right":
                    action = "spin_r"
                else:
                    action = "spin_r"  # default spin direction

            print(f"Sensors: {state}  {action}")
            drive.drive(action)

    except KeyboardInterrupt:
        print("Line following interrupted")
    finally:
        drive.stop()

    """
    try:
        while True:
            L, M, R = sensor.read()
            action = FEEDBACK_TABLE.get((L, M, R))
            print(f"Sensors: {(L, M, R)}  {action}")
            drive.drive(action)

    except KeyboardInterrupt:
        print("Line following interrupted")
    finally:
        drive.stop()
     """   

if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Could not connect to pigpio daemon.")
        exit()

    print("Select a functionality:")
    print("1.Flower Power")
    print("2.IR Testing")
    print("3.Line Following")

    input = input("Enter 1, 2, or 3:")

    try:
        if input == "1":
            flower_power(io)
        elif input == "2":
            ir_test(io)
        elif input == "3":
            line_following(io)
    finally:
        io.stop()
        print("Pigpio stopped.")
