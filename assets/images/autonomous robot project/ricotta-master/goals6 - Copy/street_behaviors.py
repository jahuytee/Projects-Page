import time
import traceback
import pigpio
from DriveSystem import DriveSystem
from Sense import LineSensor
from AngleSensor import AngleSensor
from proximitysensor import ProximitySensor

class Behaviors:
    THRESHOLD_HIGH = 0.63
    THRESHOLD_LOW = 0.37
    PULL_FORWARD_DURATION = 0.36
    def __init__(self, io, drive, sensor, AngleSensor):
        self.drive = drive
        self.sensor = sensor
        self.AngleSensor = AngleSensor
    

        t_intersection = 0.17
        t_end = 0.2
        t_side = 0.1
        side_threshold = 0.1

        # Timers and states for detectors
        self.tlast = time.time()

        self.intersection_level = 0.0
        self.intersection_state = False
        self.t_intersection = t_intersection

        self.end_level = 0.0
        self.end_state = False
        self.t_end = t_end

        self.side_level = 0.0
        self.side_state = "center"
        self.t_side = t_side
        self.side_threshold = side_threshold

        # Turning Behavior
        self.turn_level = 0.0
        self.t_spin = 0.1
        self.on_path = False

    def reset_filters(self):
        self.intersection_level = 0.0
        self.intersection_state = False
        self.end_level = 0.0
        self.end_state = False
        self.side_level = 0.0
        self.side_state = "center"

    def raw_side_estimate(self, reading):
        if reading == [1, 0, 0]:
            return 1.0
        elif reading == [0, 0, 1]: 
            return -1.0
        elif reading == [1, 1, 0]:
            return 0.5
        elif reading == [0, 1, 1]:
            return -0.5
        elif reading == [0, 1, 0] or reading == [1, 1, 1]:
            return 0.0
        else:
            return 0.0
        
    def update_detectors(self, L, M, R):
        tnow = time.time()
        dt = tnow - self.tlast
        self.tlast = tnow

        # INTERSECTION DETECTOR
        raw_intersection = 1.0 if (L, M, R) == (1, 1, 1) else 0.0
        self.intersection_level += dt / self.t_intersection * (raw_intersection - self.intersection_level)
        if self.intersection_level > self.THRESHOLD_HIGH:
            self.intersection_state = True
        elif self.intersection_level < self.THRESHOLD_LOW:
            self.intersection_state = False

        # END DETECTOR
        if self.side_state == "center":
            if (L,M,R) == (0,0,0):
                raw_end = 1.0
            else:
                raw_end = 0.0
        else:
            raw_end = 0.0        
        self.end_level += dt / self.t_end * (raw_end - self.end_level)
        if self.end_level > self.THRESHOLD_HIGH:
            self.end_state = True
        elif self.end_level < self.THRESHOLD_LOW:
            self.end_state = False

        # SIDE ESTIMATOR
        if (L, M, R) != (0, 0, 0):
            reading = [L, M, R]
            raw_side = self.raw_side_estimate(reading)
            self.side_level += dt / self.t_side * (raw_side - self.side_level)
            if self.side_level > self.side_threshold:
                self.side_state = "right"
            elif self.side_level < -self.side_threshold:
                self.side_state = "left"
            else:
                self.side_state = "center"

    def follow_line(self):
        print("Starting line-following behavior...")

        while True:
            L, M, R = self.sensor.read()
            self.update_detectors(L, M, R)

            if self.intersection_state:
                self.drive.stop()
                print("Intersection detected!")
                self.reset_filters()
                return "intersection"

            elif self.end_state:
                self.drive.stop()
                print("End of street detected!")
                self.reset_filters()
                return "end"

            elif (L, M, R) == (0, 0, 0):
                print(f"Tape lost. The estimated side is: {self.side_state}")
                if self.side_state == "left":
                    self.drive.drive("spin_r")
                elif self.side_state == "right":
                    self.drive.drive("spin_l")
                else:
                    self.drive.drive("straight")

            else:
                feedback = {
                    (0, 1, 0): "straight",
                    (0, 1, 1): "turn_r",
                    (0, 0, 1): "hook_r",
                    (1, 1, 0): "turn_l",
                    (1, 0, 0): "hook_l",
                    (1, 1, 1): "straight",
                }
                action = feedback.get((L, M, R), "straight")
                self.drive.drive(action)

            time.sleep(0.01)

    def pull_forward(self):
        t0 = time.time()
        self.reset_filters()  # Reset any previous detector values
        self.tlast = time.time()
        self.end_level = 0.0
        self.end_state = False

        while True:

            tnow = time.time()
            dt = tnow - self.tlast
            self.tlast = tnow
            
            L,M,R = self.sensor.read()
            if M == 1:
                raw_end = 1.0
            else:
                raw_end = 0.0
                   
            self.end_level += dt / self.t_end * (raw_end - self.end_level)
            if self.end_level > self.THRESHOLD_HIGH:
                self.end_state = True
            elif self.end_level < self.THRESHOLD_LOW:
                self.end_state = False

            tnow = time.time()
            self.drive.drive("straight")
            if tnow >= t0 + self.PULL_FORWARD_DURATION:
                break
            time.sleep(0.01)
        self.drive.stop()
        print("Finished pull-forward.")
        print(self.end_level)
        
        result = self.end_level > self.THRESHOLD_HIGH
        
        self.reset_filters()

        return result

    def turning_behavior(self, choice):
        print(f"Starting turning behavior: {choice}")
        
        t_start = time.time()

        # Choose turning direction
        if choice == "left":
            self.drive.drive("spin_l")
            start_time = time.time()
        elif choice == "right":
            self.drive.drive("spin_r")
            start_time = time.time()
        else:
            print("A valid turn direction was not selected")
            return 
        
        prev_angle = self.AngleSensor.read_angle()
        cumulative_angle = 0
        

        # Phase 1 Leave the original street
        print("Phase 1: Looking to leave the original street...")
        self.tlast = time.time()
        self.turn_level = 1.0  # Assume starting on tape

        left_street = False  # Set once we've left

        while not left_street:
            curr_angle = self.AngleSensor.read_angle()
            delta = curr_angle - prev_angle

            if delta > 180:
                delta -= 360
            if delta < -180:
                delta += 360

            if choice == "left":
                cumulative_angle += delta
            else:
                delta = -delta
                
            prev_angle = curr_angle

            L, M, R = self.sensor.read()
            raw = 1.0 if M == 1 else 0.0

            tnow = time.time()
            dt = tnow - self.tlast
            self.tlast = tnow

            self.turn_level += dt / self.t_spin * (raw - self.turn_level)

            if self.turn_level < self.THRESHOLD_LOW:
                left_street = True
                print(" Left the original street.")
            

        # Phase 2 Find the new street 
        print("Phase 2: Searching for new street...")

        found_new_street = False

        while not found_new_street:
            curr_angle = self.AngleSensor.read_angle()
            delta = curr_angle - prev_angle

            if delta > 180:
                delta -= 360
            if delta < -180:
                delta += 360

            if choice == "left":
                cumulative_angle += delta
            else:
                cumulative_angle -= delta
            prev_angle = curr_angle
            
            L, M, R = self.sensor.read()
            raw = 1.0 if M == 1 else 0.0

            tnow = time.time()
            dt = tnow - self.tlast
            self.tlast = tnow

            self.turn_level += dt / self.t_spin * (raw - self.turn_level)

            if self.turn_level > 0.63:
                found_new_street = True
                print("Found the new street.")
            

        # Done turning
        self.drive.stop()
        if choice == "right":
            cumulative_angle = -cumulative_angle
            
        t_end = time.time()
            
        elapsed = t_end - t_start
        print(f"The turn took {elapsed} seconds")
        print(f"The angle turned is {cumulative_angle} degrees")
            
        return round(cumulative_angle/45)
        

    def realign(self, choice):
        if choice == "left":
            self.drive.drive("spin_r")
        elif choice == "right":
            self.drive.drive("spin_l")
        while True:
            (L,M,R) = self.sensor.read()
            if (L,M,R) == (0,1,0):
                self.drive.stop()
                break
                    

        
        


if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Could not connect to pigpio daemon.")
        exit()

    try:
        drive = DriveSystem(io)
        sensor = LineSensor(io)
        behaviors = Behaviors(io, drive, sensor)

        print("Simple Brain started.")
        
        while True:
            choice = input("Drive Straight, left, or right: ").lower()

            if choice in ("left", "right"):
                behaviors.turning_behavior(choice)

            elif choice == "straight":
                print("Following line")
                result = behaviors.follow_line()

                if result == "intersection":
                    print("Intersection reached.")
                    behaviors.pull_forward(0.4)

                elif result == "end":
                    print("Reached end of the street.")

                else:
                    print("Unknown line-following result:", result)

            else:
                print("Invalid input. Please enter 'straight', 'left', or 'right'.")



    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Exiting...")

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    finally:
        drive.stop()
        io.stop()
        print("Robot stopped. Brain shutdown :3")



