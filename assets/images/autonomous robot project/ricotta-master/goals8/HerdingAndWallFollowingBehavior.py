import time
import pigpio
from DriveSystem import DriveSystem
from proximitysensor import ProximitySensor
import threading


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

        left_close = d_left < 20
        mid_far = d_mid > 20
        mid_mid =  10 <= d_mid <= 20
        mid_close = d_mid < 10
        right_close = d_right < 20

        # 12 explicit cases
        if not left_close and mid_far and not right_close:
            # All clear, go straight forward
            drive.drive("straight")
        elif left_close and mid_far and not right_close:
            # Obstacle left, go forward and steer right
            drive.drive("steer_r")
        elif not left_close and mid_far and right_close:
            # Obstacle right, go forward and steer left
            drive.drive("steer_l")
        elif left_close and mid_far and right_close:
            # obstacles both sides, go forward
            drive.drive("straight")
        elif not left_close and mid_mid and not right_close:
            # all clear, but something ahead at medium distance, so stop
            drive.stop()
        elif left_close and mid_mid and not right_close:
            # Obstacle left, medium ahead, veer right (or stop)
            drive.drive("steer_r")
        elif not left_close and mid_mid and right_close:
            # Obstacle right, medium ahead, veer left (or stop)
            drive.drive("steer_l")
        elif left_close and mid_mid and right_close:
            # Obstacles both sides, medium ahead, spin left (or stop)
            drive.drive("spin_l")
        elif not left_close and mid_close and not right_close:
            # All clear on sides, but close ahead, back up straight
            drive.drive("straight", reverse=True)
        elif left_close and mid_close and not right_close:
            # Obstacle left, close ahead, back up and steer right
            drive.drive("steer_r", reverse=True)
        elif not left_close and mid_close and right_close:
            # Obstacle right, close ahead, back up and steer left
            drive.drive("steer_l", reverse=True)
        elif left_close and mid_close and right_close:
            # Obstacles everywhere, back up 
            drive.drive("straight", reverse=True)
        else:
            # Fallback safety
            drive.stop()

        time.sleep(0.05)
            

class SharedMode:
    def __init__(self):
        self.mode = 0  # 0=stop, 1=discrete, 2=continuous, -1=quit
        self.lock = threading.Lock()

def ui(shared):
    while True:
        cmd = input("Enter command (discrete/continuous/stop/quit): ").strip().lower()
        with shared.lock:
            if cmd == "discrete":
                shared.mode = 1
            elif cmd == "continuous":
                shared.mode = 2
            elif cmd == "stop":
                shared.mode = 0
            elif cmd == "quit":
                shared.mode = -1
                break
            else:
                print("Unknown command.")

def wall_following_behavior(drive, prox, shared):
    d0 = 30 # Desired distance from wall in cm
    while True:
        with shared.lock:
            mode = shared.mode
        if mode < 0:
            print("Quitting wall following loop.")
            break
        elif mode == 0:
            drive.stop()
            time.sleep(0.05)
            continue
        elif mode == 1:
            d_left, d_mid, d_right = prox.read_all()
            if d_left is None:
                drive.stop()
                print("No left sensor reading. Stopping.")
                continue
            if d_mid is not None and d_mid < 20:
                drive.stop()
                print("Obstacle ahead! Stopping.")
                continue
            if d_left < 20 or d_left > 40:
                drive.stop()
                print("Too far from wall bounds. Stopping.")
                break
            error = d_left - d0
            print(f"Distance from wall: {d_left} cm, Error: {error} cm")
            if abs(error) <= 3:
                drive.drive("straight")
            elif  3 < error <= 7:
                drive.drive("veer_l")
            elif 7 < error <= 10:
                drive.drive("steer_l")
            elif -10 <= error < -7: 
                drive.drive("steer_r")
            elif -7 < error < -3:
                drive.drive("veer_r")
            else:
                drive.stop()  # fallback safety
            time.sleep(0.05)
        elif mode == 2:
            d_left, d_mid, d_right = prox.read_all()
            if d_left is None:
                drive.stop()
                print("No left sensor reading. Stopping.")
                continue
            if d_mid is not None and d_mid < 20:
                drive.stop()
                print("Obstacle ahead! Stopping.")
                continue
            if d_left < 20 or d_left > 40:
                drive.stop()
                print("Too far from wall bounds. Stopping.")
                break

            error = d_left - d0
            pwm_L = 0.8035347173 - 0.01189577199 * error
            pwm_R = 0.7717609744 + 0.008546060135 * error

            # Clamp PWM values to [0, 1] for safety
            pwm_L = max(0, min(1, pwm_L))
            pwm_R = max(0, min(1, pwm_R))

            drive.pwm(pwm_L, pwm_R)
            print(f"Continuous mode: error={error:.2f}, pwm_L={pwm_L:.3f}, pwm_R={pwm_R:.3f}")
            time.sleep(0.05)
        else:
            drive.stop()
            time.sleep(0.05)


if __name__ == "__main__":

    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon.")
        exit()

    drive = DriveSystem(io)
    prox = ProximitySensor(io)
    shared = SharedMode()
    ui_thread = threading.Thread(target=ui, args=(shared,), daemon=True)
    ui_thread.start()
    try:
        wall_following_behavior(drive, prox, shared)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        drive.stop()
        prox.shutdown()
        io.stop()
