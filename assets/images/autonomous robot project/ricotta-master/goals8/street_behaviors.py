import time
import traceback
import pigpio
from DriveSystem import DriveSystem
from Sense import LineSensor
from AngleSensor import AngleSensor
from proximitysensor import ProximitySensor
import math

class Behaviors:
    THRESHOLD_HIGH = 0.55
    THRESHOLD_LOW = 0.37
    PULL_FORWARD_DURATION = 0.36
    PULL_FORWARD_THRESHOLD = 0.41
    
    # Quadratic model coefficients: y = -14.88x^2 + 162.92x - 40.47
    # Updated regression model for time-to-angle prediction
    TURN_COEFF_A = -14.88   # quadratic term
    TURN_COEFF_B = 162.92   # linear term
    TURN_COEFF_C = -40.47   # constant term
    TURN_SCALE_FACTOR = 1.14  # Scale factor to correct 315° to 360°
    
    # Weights for turn estimation
    TIME_WEIGHT = 0.4      # Weight for time-based prediction
    ANGLE_WEIGHT = 0.6     # Weight for magnetometer reading
    
    def __init__(self, io, drive, sensor, AngleSensor, proximity_sensor=None):
        self.drive = drive
        self.sensor = sensor
        self.AngleSensor = AngleSensor
        self.proximity_sensor = proximity_sensor  # Add proximity sensor
    
        # Deadend detection parameters
        self.lost_line_time = 0
        self.MAX_LOST_LINE_TIME = 1.0  # Maximum time to spend hunting for a lost line before treating as deadend
        
        t_intersection = 0.17
        t_end = 0.35
        t_side = 0.3
        side_threshold = 0.30  # Increased from 0.05 to be more tolerant of wobbling

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
        self.lost_line_time = 0

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
        if self.end_level > 0.8:
            self.end_state = True
        elif self.end_level < self.THRESHOLD_LOW:
            self.end_state = False

        # SIDE ESTIMATOR
        if (L, M, R) != (0, 0, 0):
            # When on a line, update side state normally
            reading = [L, M, R]
            raw_side = self.raw_side_estimate(reading)
            self.side_level += dt / self.t_side * (raw_side - self.side_level)
            if self.side_level > self.side_threshold:
                self.side_state = "right"
            elif self.side_level < -self.side_threshold:
                self.side_state = "left"
            else:
                self.side_state = "center"
        elif self.end_level < 0.3:
            # When line is lost but not yet confirmed as end, keep side state stable
            # This gives end detector time to accumulate evidence
            pass

    def follow_line(self, block_threshold_cm=10, clear_threshold_cm=20):
        """
        Follow the line, but stop if the forward proximity sensor detects an obstacle closer than block_threshold_cm.
        Wait until the path is clear (distance > clear_threshold_cm) before resuming.
        """
        self.lost_line_time = 0  # Reset timer when starting to follow line
        waiting_for_clear = False

        while True:
            # --- Scan ahead for obstacles ---
            blocked = False
            if self.proximity_sensor is not None:
                distances = self.proximity_sensor.read_all()
                middle = distances[1]
                if middle is not None:
                    if not waiting_for_clear and middle < block_threshold_cm:
                        print(f"Obstacle detected ahead at {middle:.1f} cm! Stopping robot before collision.")
                        self.drive.stop()
                        waiting_for_clear = True
                        # Do not update detectors while stopped
                        time.sleep(0.05)
                        continue
                    elif waiting_for_clear:
                        if middle > clear_threshold_cm:
                            print(f"Path ahead is now clear at {middle:.1f} cm. Resuming line following.")
                            waiting_for_clear = False
                        else:
                            # Still blocked, keep waiting
                            continue

            if waiting_for_clear:
                # If for some reason we get here, just wait
                self.drive.stop()
                time.sleep(0.05)
                continue

            # --- Normal line following logic ---
            L, M, R = self.sensor.read()
            self.update_detectors(L, M, R)

            if self.intersection_state:
                self.drive.stop()
                print("Intersection detected!")
                self.reset_filters()
                self.lost_line_time = 0
                print(self.intersection_level)
                return "intersection"

            elif self.end_state:
                self.drive.stop()
                print("End of street detected!")
                self.reset_filters()
                self.lost_line_time = 0
                return "end"

            elif (L, M, R) == (0, 0, 0):
                print(f"Tape lost. The estimated side is: {self.side_state}")
                # Increment lost line timer
                self.lost_line_time += 0.01  # Assumes 10ms loop timing
                if self.lost_line_time > self.MAX_LOST_LINE_TIME:
                    self.drive.stop()
                    print(f"Line lost for {self.lost_line_time:.1f} seconds - treating as deadend!")
                    self.reset_filters()
                    self.lost_line_time = 0
                    return "end"
                # Attempt to find the line based on side state
                if self.side_state == "left":
                    self.drive.drive("spin_r")
                elif self.side_state == "right":
                    self.drive.drive("spin_l")
                else:
                    self.drive.drive("straight")
            else:
                # We're on a line, reset lost line timer
                self.lost_line_time = 0
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
        print(self.end_level)
        
        result = self.end_level > self.PULL_FORWARD_THRESHOLD
        
        self.reset_filters()

        return result

    def predict_turn_time(self, target_angle):
        """
        Calculate turn time using quadratic formula to solve:
        target_angle = -14.88x² + 162.92x - 40.47
        """
        a = self.TURN_COEFF_A
        b = self.TURN_COEFF_B
        c = self.TURN_COEFF_C - target_angle
        
        # Quadratic formula: (-b ± sqrt(b² - 4ac)) / (2a)
        # Since a is negative and we want positive time, use the smaller root
        discriminant = b*b - 4*a*c
        if discriminant < 0:
            print(f"Warning: No real solution for angle {target_angle}°")
            return 1.0  # Default fallback time
            
        time = (-b - math.sqrt(discriminant)) / (2*a)
        return max(0.0, time)

    def predict_angle_from_time(self, time):
        """
        Predict angle using quadratic model with scaling correction:
        angle = (-14.88x² + 162.92x - 40.47) * scale_factor
        
        The scale factor corrects for the systematic underestimation
        of large angles, particularly around 360 degrees.
        """
        base_angle = (self.TURN_COEFF_A * time**2 + 
                     self.TURN_COEFF_B * time + 
                     self.TURN_COEFF_C)
        
        # Apply scaling correction
        # Scale more aggressively for larger angles
        if base_angle > 180:
            scale = self.TURN_SCALE_FACTOR
        else:
            # Linear interpolation of scale factor based on angle
            scale = 1.0 + (base_angle / 180.0) * (self.TURN_SCALE_FACTOR - 1.0)
            
        return base_angle * scale

    def turning_behavior(self, choice):
        print(f"Starting turning behavior: {choice}")
        
        # Reset turn detector
        self.turn_level = 0.0
        self.tlast = time.time()
        
        # Track angle using magnetometer
        t_start = time.time()
        prev_angle = self.AngleSensor.read_angle()
        cumulative_angle = 0.0
        
        # Two-phase approach
        # Phase 1: Get off the current line
        phase = 1
        
        # Start turning
        if choice == "left":
            self.drive.drive("spin_l")
        elif choice == "right":
            self.drive.drive("spin_r")
            
        while True:
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
            
            # Read line sensor
            L, M, R = self.sensor.read()
            
            # Determine if we're on a line
            if phase == 1:
                # Phase 1: Looking to get OFF the line
                raw = 0.0 if M == 1 else 1.0  # Inverted logic to detect when OFF the line
            else:
                # Phase 2: Looking to get back ON a line
                raw = 1.0 if M == 1 else 0.0
                
            # Update turn detector
            tnow = time.time()
            dt = tnow - self.tlast
            self.tlast = tnow
            self.turn_level += dt / self.t_spin * (raw - self.turn_level)
            
            # State transitions
            if phase == 1 and self.turn_level > 0.63:
                # Successfully off the line, transition to phase 2
                phase = 2
                # Reset detection level for new phase
                self.turn_level = 0.0
            elif phase == 2 and self.turn_level > 0.63:
                # Found new line
                break
                
        # Done turning
        self.drive.stop()
        
        # Calculate time and time-based prediction before realignment
        t_end = time.time()
        elapsed = t_end - t_start
        time_based_angle = self.predict_angle_from_time(elapsed)
        if choice == "right":
            time_based_angle = -time_based_angle
            
        # Realign to center of line
        self.realign(choice)
        
        # Get final magnetometer reading after realignment
        if choice == "right":
            magnetometer_angle = -abs(cumulative_angle)
        else:
            magnetometer_angle = abs(cumulative_angle)
            
        # Calculate weighted average of the two measurements
        weighted_angle = (self.TIME_WEIGHT * time_based_angle + 
                        self.ANGLE_WEIGHT * magnetometer_angle)
        
        # Round the average to nearest 45° increment
        num_increments = round(weighted_angle / 45.0)
        final_angle = num_increments * 45.0
        
        # Data collection mode - print all measurements
        print("\nTurn Analysis:")
        print(f"Elapsed Time: {elapsed:.3f}s")
        print(f"Time-based prediction: {time_based_angle:.1f}°")
        print(f"Magnetometer reading: {magnetometer_angle:.1f}°")
        print(f"Weighted average: {weighted_angle:.1f}°")
        print(f"Rounded to: {final_angle:.1f}° ({num_increments} × 45°)")
            
        # Return both the number of 45-degree steps and the weighted average angle
        return num_increments, magnetometer_angle

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
                    
    def check_blockage(self, threshold_cm=40):
        """
        Returns True if the forward-facing (middle) ultrasound sensor detects an obstacle closer than threshold_cm.
        """
        if self.proximity_sensor is None:
            return False  # If no sensor, assume not blocked
        distances = self.proximity_sensor.read_all()
        middle = distances[1]
        if middle is not None and middle < threshold_cm:
            print(f"Blockage detected ahead! Distance: {middle:.1f} cm < {threshold_cm} cm")
            return True
        return False

if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Could not connect to pigpio daemon.")
        exit()

    try:
        drive = DriveSystem(io)
        sensor = LineSensor(io)
        anglesensor = AngleSensor(io)
        proximity_sensor = ProximitySensor(io)
        behaviors = Behaviors(io, drive, sensor, anglesensor, proximity_sensor)

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
                    behaviors.pull_forward()

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



