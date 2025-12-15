import time
import pigpio
from DriveSystem import DriveSystem
from Sense import LineSensor

class LineFollower:
    def __init__(self, io, drive, sensor):
        
        self.drive = drive
        self.sensor = sensor
        
        
        t_intersection=0.17 
        t_end=0.2 
        t_side=0.1
        side_threshold=0.1

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

    def raw_side_estimate(self, L, M, R):
        if L == 1 and M == 0 and R == 0:
            return 1.0
        elif L == 0 and M == 0 and R == 1:
            return -1.0
        elif L == 1 and M == 1 and R == 0:
            return 0.5
        elif L == 0 and M == 1 and R == 1:
            return -0.5
        elif L == 0 and M == 1 and R == 0:
            return 0.0
        elif L == 1 and M == 1 and R == 1:
            return 0.0

    def update_detectors(self, L, M, R):
        tnow = time.time()
        dt = tnow - self.tlast
        self.tlast = tnow

        #INTERSECTION DETECTOR
        raw_intersection = 1.0 if (L, M, R) == (1, 1, 1) else 0.0
        self.intersection_level += dt / self.t_intersection * (raw_intersection - self.intersection_level)
        if self.intersection_level > 0.63:
            self.intersection_state = True
        elif self.intersection_level < 0.37:
            self.intersection_state = False

        #END DETECTOR
        if self.side_state == "center":
            raw_end = 1.0 if (L, M, R) == (0, 0, 0) else 0.0
        else:
            raw_end = 0.0
        self.end_level += dt / self.t_end * (raw_end - self.end_level)
        if self.end_level > 0.63:
            self.end_state = True
        elif self.end_level < 0.37:
            self.end_state = False

        #SIDE ESTIMATOR
        if (L,M,R) != (0,0,0):
            raw_side = self.raw_side_estimate(L, M, R)
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
                return "intersection"

            elif self.end_state:
                self.drive.stop()
                print("End of street detected!")
                return "end"

            elif (L, M, R) == (0, 0, 0):
                print(f"Tape lost. The estimated side is: {self.side_state}")
                # Use side estimator when tape is lost
                if self.side_state == "left":
                    self.drive.drive("spin_r")
                elif self.side_state == "right":
                    self.drive.drive("spin_l")
                else:
                    self.drive.drive("straight")
                    
            else:
                # Default feedback behavior
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
                

class PullForward:
    def __init__(self, drive, duration):
        self.drive = drive
        self.duration = duration

    def pull_forward(self):
        print(f"Executing pull-forward for {self.duration:.2f}")

        t0 = time.time()
        while True:
            tnow = time.time()

            self.drive.drive("straight")

            if tnow >= t0 + self.duration:
                break
        self.drive.stop()
        print("Finished pull-forward.")

              
class TurningBehavior:
    def __init__(self, drive, sensor, choice):
        
        self.drive = drive
        self.sensor = sensor
        self.choice = choice
        
    
        self.tlast = time.time()
        self.turn_level = 0.0
        self.t_spin = 0.1
        self.on_path = False
        
    def turn(self):
        if self.choice == "left":
            self.drive.drive("spin_l")
            start_time = time.time()
        elif self.choice == "right":
            self.drive.drive("spin_r")
            start_time = time.time()
        else:
            print("A valid turn direction was not selected")
        
        L,M,R = self.sensor.read()
        if M == 1:
            print("We are starting on the tape, waiting to go off")
            while True:
                L,M,R = self.sensor.read()
                if M == 0:
                    print("We are now off tape, starting turn detection")
                    break
                    
        self.tlast = time.time()   
        self.turn_level = 0.0         
        
        while True:
            L,M,R = self.sensor.read()
            if M == 1:
                raw = 1.0
            else:
                raw = 0.0
            tnow = time.time()
            dt = tnow - self.tlast
            self.tlast = tnow
            self.turn_level += dt/self.t_spin * (raw-self.turn_level)
            
            if self.turn_level > 0.63:
                self.on_path = True
                break
            elif self.turn_level < 0.37:
                self.on_path = False
            
        self.drive.stop()
        end_time = time.time()
        elapsed_time = end_time - start_time
        
        print(f"Finished {self.choice} @ {elapsed_time} sec)")
                  
        
              
            
        
      
                
                
                
if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Could not connect to pigpio daemon.")
        exit()
         
         
    try:
        # Define GPIO pins for IR sensors
        PIN_IR_LEFT   = 14
        PIN_IR_MIDDLE = 15
        PIN_IR_RIGHT  = 18

        # Set up motor driver and sensors
        left_motor_pins = (8, 7)
        right_motor_pins = (6, 5)
        drive = DriveSystem(io, left_motor_pins, right_motor_pins)
        sensor = LineSensor(io, PIN_IR_LEFT, PIN_IR_MIDDLE, PIN_IR_RIGHT)

        # Create LineFollower instance
        follower = LineFollower(io,drive,sensor)

        # Run line-following loop
        result = follower.follow_line()

        if result == "intersection":
            forward = PullForward(drive, duration=0.5)
            forward.pull_forward()
            turning = TurningBehavior(drive,sensor)
            turning.turn
        elif result == "end":
            print("At the end of the road")

    except KeyboardInterrupt:
        print("Divan")

    finally:
        drive.stop()
        io.stop()
        print("Shutdown complete.")   
                
            
            
