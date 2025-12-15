import time
import pigpio
from DriveSystem import DriveSystem
from proximitysensor import ProximitySensor


def herding_behavior(drive, prox):
    last_trigger_time = time.time()
    prox.trigger_all()
    time.sleep(0.05)

    while True:
        now = time.time()
        if now - last_trigger_time > 0.05:
            prox.trigger_all()
            last_trigger_time = now

        d_left, d_mid, d_right = prox.read_all()

        # Categorize the distances
        forward_mode = "stop"
        turn_mode = None

        # Forward/backward logic
        # first check middle sensor
        if d_mid is not None:
            if d_mid > 20:
                forward_mode = "forward"
            elif d_mid < 10:
                forward_mode = "backward"

        # Turning logic
        # then check left and right
        if d_left is not None and d_right is not None:
            if d_left < 20 and d_right >= 20:
                turn_mode = "right"
            elif d_right < 20 and d_left >= 20:
                turn_mode = "left"
            elif d_right < 20 and d_left < 20:
                turn_mode = "spin"

        # then we decide a movement base off of the 12 cases
        # e.g. if left is close and right is far, we turn right
        if forward_mode == "forward":
            if turn_mode == "left":
                drive.drive("steer_r")
            elif turn_mode == "right":
                drive.drive("steer_l")
            elif turn_mode == "spin":
                drive.drive("spin_l")
            else:
                drive.drive("straight")

        elif forward_mode == "backward":
            if turn_mode == "left":
                drive.drive("steer_r", reverse=True)
            elif turn_mode == "right":
                drive.drive("steer_l", reverse=True)
            elif turn_mode == "spin":
                drive.drive("spin_l")
            else:
                drive.drive("straight", reverse=True)

        else:
            drive.stop()

        time.sleep(0.05)



if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon.")
        exit()

    try:
        herding_behavior(drive, prox)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        drive.stop()
        io.stop()
