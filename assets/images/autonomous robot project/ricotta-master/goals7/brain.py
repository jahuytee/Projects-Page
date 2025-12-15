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
    print("Attempting initial alignment to road...")
    has_street = False  # Initialize has_street
    while True:
        L, M, R = behaviors.sensor.read()
        if (L, M, R) == (0, 0, 0):
            print("Off-road — performing U-turn.")
            behaviors.turning_behavior("left")
            result = behaviors.follow_line()
            if result == "intersection":
                print("Back at intersection.")
                has_street = behaviors.pull_forward()
                inter = map.getintersection(map.x, map.y)
                if has_street:
                    map.setstreet(map.x, map.y, map.heading, STATUS.UNEXPLORED)
                else:
                    map.setstreet(map.x, map.y, map.heading, STATUS.NONEXISTENT)
                    

               

                
        else:
            print("On-road — following line forward.")
            result = behaviors.follow_line()
            if result == "intersection":
                print("Reached first intersection.")
                has_street = behaviors.pull_forward()
                inter = map.getintersection(map.x, map.y)
                if has_street:
                    map.setstreet(map.x, map.y, map.heading, STATUS.UNEXPLORED)
                else:
                    map.setstreet(map.x, map.y, map.heading, STATUS.NONEXISTENT)

                
            elif result == "end":
                print("Dead end detected - executing U-turn")
                street_in_front = map.markdeadend(behaviors)
                if street_in_front:
                    inter = map.getintersection(map.x, map.y)
                    x, y, h = map.pose()
                    map.setstreet(x, y, h, STATUS.UNEXPLORED)
                    for delta in [-1, 1, -3, 3]:
                        diag_heading = (h + delta) % 8
                        if inter.streets[diag_heading] == STATUS.UNKNOWN:
                            map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                else:
                    map.setstreet(x, y, h, STATUS.NONEXISTENT)
                    for delta in [-3, 3]:
                        diag_heading = (h + delta) % 8
                        if inter.streets[diag_heading] == STATUS.UNKNOWN:
                            map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)

                break

        inter = map.getintersection(map.x, map.y)
        x, y, h = map.pose()
        if has_street:
            for delta in [-1, 1, -3, 3]:
                diag_heading = (h + delta) % 8
                if inter.streets[diag_heading] == STATUS.UNKNOWN:
                    map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
            map.setstreet(x,y,h,STATUS.UNEXPLORED)
        else:
            for delta in [-3, 3]:
                diag_heading = (h + delta) % 8
                if inter.streets[diag_heading] == STATUS.UNKNOWN:
                    map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)

        break

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
        choice_is_load = False
        if choice == "load":
            choice_is_load = True

        print("Simple Brain started.")
        last_pose = None
        if not choice_is_load:
            align_to_road(behaviors, map)
        

        while True:
            map.showwithrobot()
            mode = input("Enter command (straight, left, right, goto, save, load, auto, quit): ").strip().lower()

            if mode == "quit":
                break

            if mode in ("left", "right"):
                turn_amt, actual_angle = behaviors.turning_behavior(mode)
                map.markturn(turn_amt, actual_angle)

            elif mode == "straight":
                result = behaviors.follow_line()
                if result == "intersection":
                    has_street = behaviors.pull_forward()
                    map.update_connection()
                    x, y, h = map.pose()
                    inter = map.intersections.get((x, y))
                    if has_street:
                        # Only set to UNEXPLORED if it was UNKNOWN
                        if inter.streets[h] == STATUS.UNKNOWN:
                            map.setstreet(x,y,h,STATUS.UNEXPLORED)
                        # Only mark diagonals as NONEXISTENT
                        for delta in [-1, 1,3,-3]:
                            diag_heading = (h + delta) % 8
                            if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                    else:
                        # If no street ahead, mark current heading as NONEXISTENT
                        map.setstreet(x, y, h, STATUS.NONEXISTENT)
                        # And mark back diagonals as NONEXISTENT
                        for delta in [-3, 3]:
                            diag_heading = (h + delta) % 8
                            if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                elif result == "end":
                    print("Dead end detected - executing U-turn")
                    map.markdeadend(behaviors)

            elif mode == "goto":
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

            elif mode == "save":
                filename = input("Enter filename to save (e.g. mymap.pkl): ").strip()
                with open(filename, 'wb') as file:
                    pickle.dump(map, file)
                print(f"Map saved to {filename}")

            elif mode == "load":
                map = prompt_and_load_map()

            elif mode == "auto":
                print("Autonomous mode: Robot is exploring the map...")
                while True:
                    map.showwithrobot()
                    x, y, current_heading = map.pose()
                    print(f"\nCurrent Position: ({x}, {y}), Heading: {current_heading}")
                    current_intersection = map.intersections.get((x, y))

                    if not current_intersection:
                        print("Error: Robot not at a valid intersection")
                        break

                    # Print current intersection state
                    print("\nCurrent intersection street states:")
                    for h in range(8):
                        status = current_intersection.streets[h]
                        print(f"Heading {h}: {status}")

                    # Step 1: Check for UNKNOWN streets
                    unknown_headings = [h for h in range(8)
                                     if current_intersection.streets[h] == STATUS.UNKNOWN]
                    print(f"\nUnknown streets at headings: {unknown_headings}")
                    
                    if unknown_headings:
                        # Find heading with smallest rotation needed
                        best_heading = min(unknown_headings, 
                                         key=lambda h: min((h - current_heading) % 8, 
                                                         (current_heading - h) % 8))
                        print(f"Best heading: {best_heading}")
                        diff = (best_heading - current_heading) % 8
                        counter_diff = (current_heading - best_heading) % 8
                        print(f"Diff: {diff}, Counter diff: {counter_diff}")
                        if diff <= counter_diff:
                            turn_direction = "right"
                            turns_needed = diff
                        else:
                            turn_direction = "left"
                            turns_needed = counter_diff
                        
                        print(f"Found unknown street at heading {best_heading}, turning {turn_direction} {turns_needed} times")
                        for _ in range(turns_needed):
                            turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                            map.markturn(turn_amt, actual_angle)
                            x, y, current_heading = map.pose()
                            print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")

                        result = behaviors.follow_line()
                        if result == "intersection":
                            has_street = behaviors.pull_forward()
                            map.update_connection()
                            x, y, h = map.pose()
                            print(f"Reached intersection - Position: ({x}, {y}), Heading: {h}")
                            print(f"Street ahead exists: {has_street}")
                            inter = map.intersections.get((x, y))
                            if has_street:
                                # Mark current street as UNEXPLORED
                                if inter.streets[h] == STATUS.UNKNOWN:
                                    map.setstreet(x, y, h, STATUS.UNEXPLORED)
                                    print(f"Marked street at heading {h} as UNEXPLORED")
                                # Mark diagonals as NONEXISTENT
                                for delta in [-1, 1, -3, 3]:
                                    diag_heading = (h + delta) % 8
                                    if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                        map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                                        print(f"Marked diagonal street at heading {diag_heading} as NONEXISTENT")
                            else:
                                # If no street ahead, mark current heading as NONEXISTENT
                                map.setstreet(x, y, h, STATUS.NONEXISTENT)
                                print(f"Marked street at heading {h} as NONEXISTENT")
                                # Mark back diagonals as NONEXISTENT
                                for delta in [-3, 3]:
                                    diag_heading = (h + delta) % 8
                                    if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                        map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                                        print(f"Marked back diagonal street at heading {diag_heading} as NONEXISTENT")
                        elif result == "end":
                            print("Dead end detected - executing U-turn")
                            map.markdeadend(behaviors)
                            x, y, current_heading = map.pose()
                            print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
                        continue

                    # Step 2: Check for UNEXPLORED streets
                    unexplored_headings = [h for h in range(8)
                                         if current_intersection.streets[h] == STATUS.UNEXPLORED]
                    print(f"\nUnexplored streets at headings: {unexplored_headings}")
                    
                    if unexplored_headings:
                        # Find heading with smallest rotation needed
                        best_heading = min(unexplored_headings,
                                         key=lambda h: min((h - current_heading) % 8,
                                                         (current_heading - h) % 8))
                        diff = (best_heading - current_heading) % 8
                        counter_diff = (current_heading - best_heading) % 8
                        if diff <= counter_diff:
                            turn_direction = "right"
                            turns_needed = diff
                        else:
                            turn_direction = "left"
                            turns_needed = counter_diff
                        
                        print(f"Found unexplored street at heading {best_heading}, turning {turn_direction} {turns_needed} times")
                        for _ in range(turns_needed):
                            turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                            map.markturn(turn_amt, actual_angle)
                            x, y, current_heading = map.pose()
                            print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")

                        result = behaviors.follow_line()
                        if result == "intersection":
                            has_street = behaviors.pull_forward()
                            map.update_connection()
                            x, y, h = map.pose()
                            print(f"Reached intersection - Position: ({x}, {y}), Heading: {h}")
                            print(f"Street ahead exists: {has_street}")
                            inter = map.intersections.get((x, y))
                            if has_street:
                                # Mark current street as CONNECTED
                                map.setstreet(x, y, h, STATUS.CONNECTED)
                                print(f"Marked street at heading {h} as CONNECTED")
                                # Mark diagonals as NONEXISTENT
                                for delta in [-1, 1, -3, 3]:
                                    diag_heading = (h + delta) % 8
                                    if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                        map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                                        print(f"Marked diagonal street at heading {diag_heading} as NONEXISTENT")
                            else:
                                # If no street ahead, mark current heading as NONEXISTENT
                                map.setstreet(x, y, h, STATUS.NONEXISTENT)
                                print(f"Marked street at heading {h} as NONEXISTENT")
                                # Mark back diagonals as NONEXISTENT
                                for delta in [-3, 3]:
                                    diag_heading = (h + delta) % 8
                                    if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                        map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                                        print(f"Marked back diagonal street at heading {diag_heading} as NONEXISTENT")
                        elif result == "end":
                            print("Dead end detected - executing U-turn")
                            map.markdeadend(behaviors)
                            x, y, current_heading = map.pose()
                            print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
                        continue

                    # Step 3: Use Dijkstra's to find nearest intersection with unknown/unexplored streets
                    found_goal = False
                    min_distance = float('inf')
                    best_goal = None
                    
                    print("\nSearching for nearest intersection with unknown/unexplored streets...")
                    # First check all intersections for unknown/unexplored streets
                    for tx, ty in map.intersections.keys():
                        inter = map.intersections[(tx, ty)]
                        unknown_or_unexplored = [h for h in range(8) if inter.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED)]
                        if unknown_or_unexplored:
                            # Calculate Manhattan distance as a simple heuristic
                            distance = abs(tx - x) + abs(ty - y)
                            print(f"Found intersection at ({tx}, {ty}) with unknown/unexplored streets at headings {unknown_or_unexplored}, distance: {distance}")
                            if distance < min_distance:
                                min_distance = distance
                                best_goal = (tx, ty)
                    
                    if best_goal:
                        tx, ty = best_goal
                        print(f"\nSetting goal to nearest intersection ({tx}, {ty}) with unknown/unexplored streets")
                        print(f"Current position: ({x}, {y}), Distance to goal: {min_distance}")
                        map.dijkstra(tx, ty)
                        found_goal = True
                        
                        print("Navigating to next unexplored area...")
                        while map.goal is not None:
                            map.showwithrobot()
                            map.step_toward_goal(behaviors)
                            x, y, current_heading = map.pose()
                            print(f"Moving to goal - Position: ({x}, {y}), Heading: {current_heading}")
                    else:
                        print("Exploration complete! No more unknown or unexplored streets.")
                        break

                
            else:
                print("Invalid input. Please enter a valid command.")

    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Shutting down.")
    except BaseException as ex:
        print(f"Ending due to exception: {repr(ex)}")
        traceback.print_exc()
    finally:
        drive.stop()
        io.stop()
        print("Robot stopped. Brain shutdown :3")
