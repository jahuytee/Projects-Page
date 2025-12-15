import time
import traceback
import pigpio
from DriveSystem import DriveSystem
from Sense import LineSensor
from street_behaviors import Behaviors
from AngleSensor import AngleSensor
from MapBuilding import PoseTracker, STATUS

if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Could not connect to pigpio daemon.")
        exit()

    try:
        drive = DriveSystem(io)
        sensor = LineSensor(io)
        anglesensor = AngleSensor(io)
        behaviors = Behaviors(io, drive, sensor, anglesensor)
        pose = PoseTracker()

        print("Simple Brain started.")

        while True:
            pose.show()
            print(f"Current pose: {pose.pose()}")

            choice = input("Drive Straight, left, or right: ").lower()

            if choice in ("left", "right"):
                current_intersection = pose.getintersection(pose.x, pose.y)
                prev_heading = pose.heading
                turn_amount = behaviors.turning_behavior(choice)
                pose.calcturn(turn_amount)
                if current_intersection.streets[pose.heading] == STATUS.UNKNOWN:
                    current_intersection.streets[pose.heading] = STATUS.UNEXPLORED
                new_heading = pose.heading
                skipped = []

                # If turning right 
                if turn_amount > 0:
                    for i in range(1, turn_amount):  
                        d = (prev_heading + i) % 8 # 
                        skipped.append(d)

                # If turning left 
                elif turn_amount < 0:
                    for i in range(-1, turn_amount, -1):  
                        d = (prev_heading + i) % 8 
                        skipped.append(d)

                # Now mark all skipped headings as NONEXISTENT if they were previously unknown or unexplored
                for d in skipped:
                    if current_intersection.streets[d] in (STATUS.UNKNOWN, STATUS.UNEXPLORED):
                        current_intersection.streets[d] = STATUS.NONEXISTENT
                        print(f"Marked direction {d} as NONEXISTENT (skipped during turn)")

            elif choice == "straight":
                print("Following line...")
                result = behaviors.follow_line()

                if result == "intersection":
                    print("Intersection reached.")
                    behaviors.pull_forward()

                    old_x, old_y, old_heading = pose.x, pose.y, pose.heading
                    pose.calcmove()

                    prev_intersection = pose.getintersection(old_x, old_y)
                    new_intersection = pose.getintersection(pose.x, pose.y)

                    # Mark the street as connected in both intersections
                    prev_intersection.streets[old_heading] = STATUS.CONNECTED
                    new_intersection.streets[(old_heading + 4) % 8] = STATUS.CONNECTED

  

                elif result == "end":
                    print("Reached end of the street.")

                    current_intersection = pose.getintersection(pose.x, pose.y)
                    current_intersection.streets[pose.heading] = STATUS.DEADEND

                    print("Performing U-turn")
                    turn_amount = behaviors.turning_behavior("left")
                    pose.calcturn(turn_amount)

                    print("Following line back to intersection")
                    result_back = behaviors.follow_line()

                    if result_back == "intersection":
                        print("Intersection reached again after U-turn")
                        behaviors.pull_forward()

                        old_x, old_y, old_heading = pose.x, pose.y, pose.heading

                        prev_intersection = pose.getintersection(old_x, old_y)
                        new_intersection = pose.getintersection(pose.x, pose.y)


                    else:
                        print("Warning: expected to reach intersection but didn't.")

                else:
                    print("Unknown line-following result:", result)

            else:
                print("Invalid input. Please enter 'straight', 'left', or 'right'.")

    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Shutting down.")

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    finally:
        drive.stop()
        io.stop()
        print("Robot stopped. Brain shutdown :3")
