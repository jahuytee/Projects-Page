import time
import traceback
import pigpio
import pickle
from DriveSystem import DriveSystem
from Sense import LineSensor
from street_behaviors import Behaviors
from AngleSensor import AngleSensor
from MapBuilding import Map, STATUS, prompt_and_load_map

def align_to_road(behaviors, map):
    """
    Align the robot with a road, move forward to reach an intersection,
    and pull forward. This is used to initialize both manual and autonomous modes.
    """
    print("Attempting initial alignment to road...")

    while True:
        L, M, R = behaviors.sensor.read()

        if (L, M, R) == (0, 0, 0):
            print("Off-road — performing U-turn.")
            turn_amt = behaviors.turning_behavior("left")
            map.calcturn(turn_amt)

            result = behaviors.follow_line()
            if result == "intersection":
                print("Back at intersection.")
                behaviors.pull_forward()
                break

        else:
            print("On-road — following line forward.")
            result = behaviors.follow_line()
            if result == "intersection":
                print("Reached first intersection.")
                behaviors.pull_forward()
                break

    map.getintersection(map.x, map.y)  # Initialize the map at this point


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

        choice = input("Start with blank map or load from file? (blank/load): ").strip().lower()
        map = prompt_and_load_map() if choice == "load" else Map()

        manual_mode = input("Would you like to manually explore the map? (yes/no): ").strip().lower() == "yes"

        print("Simple Brain started.")
        last_pose = None

        if manual_mode:
            print("Attempting initial alignment to road...")

            align_to_road(behaviors, map)
            print("Manual mode: You can now drive the robot.")

        while True:
            map.showwithrobot()
            if manual_mode:
                choice = input("Drive (straight, left, right, goto, save, load): ").lower()

                if choice in ("left", "right"):
                    turn_amt = behaviors.turning_behavior(choice)
                    map.calcturn(turn_amt)
                    map.markturn(turn_amt)

                elif choice == "straight":
                    result = behaviors.follow_line()
                    if result == "intersection":
                        has_street = behaviors.pull_forward()
                        map.update_connection()

                        x, y, h = map.pose()
                        inter = map.intersections.get((x, y))
                        if inter and has_street:
                            if inter.streets[h] == STATUS.UNKNOWN:
                                map.setstreet(x, y, h, STATUS.UNEXPLORED)
                            else:
                                map.setstreet(x, y, h, STATUS.CONNECTED)
                        elif inter:
                            map.setstreet(x, y, h, STATUS.NONEXISTENT)

                        if has_street:
                            for delta in [-1, 1]:
                                diag_heading = (h + delta) % 8
                                if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                    map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                                    print(f"Marking diagonal street at heading {diag_heading} from ({x}, {y}) as NONEXISTENT")

                    elif result == "end":
                        map.markdeadend(behaviors)

                elif choice == "goto":
                    try:
                        xgoal = int(input("Enter goal x: "))
                        ygoal = int(input("Enter goal y: "))
                        if not map.has_intersection(xgoal, ygoal):
                            print(f"Intersection ({xgoal}, {ygoal}) does not exist in map.")
                            continue
                        map.dijkstra(xgoal, ygoal)
                        print("Goal set. Navigating...")
                        while map.goal is not None:
                            map.showwithrobot()
                            map.step_toward_goal(behaviors)
                    except ValueError:
                        print("Invalid goal input.")

                elif choice == "save":
                    filename = input("Enter filename to save (e.g. mymap.pkl): ").strip()
                    with open(filename, 'wb') as file:
                        pickle.dump(map, file)
                    print(f"Map saved to {filename}")

                elif choice == "load":
                    map = prompt_and_load_map()

                else:
                    print("Invalid input. Please enter a valid command.")

                continue

            # Autonomous exploration mode
            if not manual_mode:
                align_to_road(behaviors, map)
                print("Autonomous mode: Robot is exploring the map.")

                while True:
                    map.showwithrobot()
                    x, y, current_heading = map.pose()
                    current_intersection = map.intersections.get((x, y))

                    if not current_intersection:
                        print("Error: Robot not at a valid intersection")
                        break

                    # Step 1: Check for UNKNOWN streets at current intersection
                    unknown_headings = []
                    for heading in range(8):
                        if current_intersection.streets[heading] == STATUS.UNKNOWN:
                            unknown_headings.append(heading)

                    if unknown_headings:
                        # Find the heading that requires the least rotation
                        current_heading = map.pose()[2]  # Get fresh heading
                        best_heading = min(unknown_headings, 
                                        key=lambda h: min(
                                            (h - current_heading) % 8,
                                            (current_heading - h) % 8
                                        ))
                        print(f"Found UNKNOWN street at heading {best_heading}, current heading {current_heading}")
                        
                        # Calculate shortest turn direction
                        diff = (best_heading - current_heading) % 8
                        if diff <= 4:
                            turn_direction = "right"
                            turns_needed = diff
                        else:
                            turn_direction = "left"
                            turns_needed = 8 - diff
                        
                        # Execute the turn
                        print(f"Turning {turn_direction} {turns_needed} steps")
                        for _ in range(turns_needed):
                            turn_amt = behaviors.turning_behavior(turn_direction)
                            map.calcturn(turn_amt)
                            map.markturn(turn_amt)

                        # Move forward after turning
                        print(f"Exploring unknown street at heading {best_heading}")
                        result = behaviors.follow_line()
                        if result == "intersection":
                            has_street = behaviors.pull_forward()
                            map.update_connection()

                            x, y, h = map.pose()
                            inter = map.intersections.get((x, y))
                            if inter and has_street:
                                if inter.streets[h] == STATUS.UNKNOWN:
                                    map.setstreet(x, y, h, STATUS.UNEXPLORED)
                                else:
                                    map.setstreet(x, y, h, STATUS.CONNECTED)
                            elif inter:
                                map.setstreet(x, y, h, STATUS.NONEXISTENT)

                            if has_street:
                                for delta in [-1, 1]:
                                    diag_heading = (h + delta) % 8
                                    if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                        map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                                        print(f"Marking diagonal street at heading {diag_heading} from ({x}, {y}) as NONEXISTENT")
                        elif result == "end":
                            map.markdeadend(behaviors)
                        continue

                    # Step 2: Check for UNEXPLORED streets
                    unexplored_headings = []
                    for heading in range(8):
                        if current_intersection.streets[heading] == STATUS.UNEXPLORED:
                            unexplored_headings.append(heading)

                    if unexplored_headings:
                        # Find the heading that requires the least rotation
                        current_heading = map.pose()[2]  # Get fresh heading
                        best_heading = min(unexplored_headings,
                                        key=lambda h: min(
                                            (h - current_heading) % 8,
                                            (current_heading - h) % 8
                                        ))
                        print(f"Found UNEXPLORED street at heading {best_heading}, current heading {current_heading}")
                        
                        # Calculate shortest turn direction
                        diff = (best_heading - current_heading) % 8
                        if diff <= 4:
                            turn_direction = "right"
                            turns_needed = diff
                        else:
                            turn_direction = "left"
                            turns_needed = 8 - diff
                        
                        # Execute the turn
                        print(f"Turning {turn_direction} {turns_needed} steps")
                        for _ in range(turns_needed):
                            turn_amt = behaviors.turning_behavior(turn_direction)
                            map.calcturn(turn_amt)
                            map.markturn(turn_amt)

                        # Move forward after turning
                        print(f"Exploring unexplored street at heading {best_heading}")
                        result = behaviors.follow_line()
                        if result == "intersection":
                            has_street = behaviors.pull_forward()
                            map.update_connection()

                            x, y, h = map.pose()
                            inter = map.intersections.get((x, y))
                            if inter and has_street:
                                map.setstreet(x, y, h, STATUS.CONNECTED)
                            elif inter:
                                map.setstreet(x, y, h, STATUS.NONEXISTENT)

                            if has_street:
                                for delta in [-1, 1]:
                                    diag_heading = (h + delta) % 8
                                    if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                        map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                                        print(f"Marking diagonal street at heading {diag_heading} from ({x}, {y}) as NONEXISTENT")
                        elif result == "end":
                            map.markdeadend(behaviors)
                        continue

                    # Step 3: Find nearest intersection with unknown/unexplored streets
                    found_goal = False
                    for target_x, target_y in map.intersections.keys():
                        target_intersection = map.intersections[(target_x, target_y)]
                        for heading in range(8):
                            if (target_intersection.streets[heading] == STATUS.UNKNOWN or 
                                target_intersection.streets[heading] == STATUS.UNEXPLORED):
                                print(f"Setting goal to intersection ({target_x}, {target_y})")
                                map.dijkstra(target_x, target_y)
                                found_goal = True
                                break
                        if found_goal:
                            break

                    if not found_goal:
                        print("Exploration complete! No more unknown or unexplored streets.")
                        break

                    # Navigate to the goal using stored direction fields
                    print("Navigating to next unexplored area...")
                    while map.goal is not None:
                        map.showwithrobot()
                        map.step_toward_goal(behaviors)

    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Shutting down.")

    except BaseException as ex:
        print(f"Ending due to exception: {repr(ex)}")
        traceback.print_exc()

    finally:
        drive.stop()
        io.stop()
        print("Robot stopped. Brain shutdown :3")
