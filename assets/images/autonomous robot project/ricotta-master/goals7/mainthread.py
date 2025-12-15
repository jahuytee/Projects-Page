# mainthread.py
import pigpio
import threading
import time
import pickle
from DriveSystem import DriveSystem
from Sense import LineSensor
from AngleSensor import AngleSensor
from street_behaviors import Behaviors
from MapBuilding import Map, STATUS
from uithread import Shared, ui
from proximitysensor import ProximitySensor


def autonomous_step(map, behaviors):
    print("Autonomous mode: Robot is exploring the map...")
    x, y, current_heading = map.pose()
    print(f"\nCurrent Position: ({x}, {y}), Heading: {current_heading}")
    current_intersection = map.intersections.get((x, y))

    if not current_intersection:
        print("Error: Robot not at a valid intersection")
        return

    # Print current intersection state
    print("\nCurrent intersection street states:")
    for h in range(8):
        status = current_intersection.streets[h]
        blocked = current_intersection.blocked[h] if hasattr(current_intersection, 'blocked') else False
        print(f"Heading {h}: {status} {'(BLOCKED)' if blocked else ''}")

    # Step 1: Check for UNKNOWN streets (skip blocked)
    unknown_headings = [h for h in range(8)
                        if current_intersection.streets[h] == STATUS.UNKNOWN 
                        and not map.is_blocked(x, y, h)]
    print(f"\nUnblocked unknown streets at headings: {unknown_headings}")
    
    if unknown_headings:
        # Find heading with smallest rotation needed
        best_heading = min(unknown_headings, 
                            key=lambda h: min((h - current_heading) % 8, 
                                            (current_heading - h) % 8))
        print(f"Best heading: {best_heading}")
        
        # Check if best heading is blocked before attempting to turn
        if map.is_blocked(x, y, best_heading):
            print(f"Best heading {best_heading} is blocked, removing from unknown_headings")
            unknown_headings.remove(best_heading)
            if not unknown_headings:
                print("No more unblocked unknown streets to explore")
                return
            # Recalculate best heading
            best_heading = min(unknown_headings,
                            key=lambda h: min((h - current_heading) % 8,
                                            (current_heading - h) % 8))
            print(f"New best heading: {best_heading}")
        
        diff = (best_heading - current_heading) % 8
        counter_diff = (current_heading - best_heading) % 8
        if diff <= counter_diff:
            turn_direction = "left"
        else:
            turn_direction = "right"
        
        print(f"Found unknown street at heading {best_heading}, turning {turn_direction}")
        prev_heading = current_heading
        max_turns = 4  # Maximum number of turns to prevent infinite loop
        turns_made = 0
        
        while current_heading != best_heading and turns_made < max_turns:
            # Check if the next heading would be blocked before turning
            next_heading = (current_heading + (1 if turn_direction == "left" else -1)) % 8
            if map.is_blocked(x, y, next_heading):
                print(f"Street at heading {next_heading} is blocked, removing from unknown_headings")
                if next_heading in unknown_headings:
                    unknown_headings.remove(next_heading)
                if not unknown_headings:
                    print("No more unblocked unknown streets to explore")
                    return
                # Recalculate best heading
                best_heading = min(unknown_headings,
                                key=lambda h: min((h - current_heading) % 8,
                                                (current_heading - h) % 8))
                print(f"New best heading: {best_heading}")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                continue

            turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
            map.markturn(turn_amt, actual_angle)
            x, y, new_heading = map.pose()
            # After turning, check blockage
            blocked = behaviors.check_blockage()
            if blocked:
                print(f"Street at heading {new_heading} is blocked, marking as blocked")
                map.set_blocked(x, y, new_heading, True)
                # Remove this heading from unknown_headings
                if new_heading in unknown_headings:
                    unknown_headings.remove(new_heading)
                if not unknown_headings:
                    print("No more unblocked unknown streets to explore")
                    return
                # Recalculate best heading
                best_heading = min(unknown_headings,
                                key=lambda h: min((h - current_heading) % 8,
                                                (current_heading - h) % 8))
                print(f"New best heading: {best_heading}")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                continue
            print(f"After turn - Position: ({x}, {y}), Heading: {new_heading}")
            # Check if we're making progress
            if new_heading == current_heading:
                print("Warning: Not making progress in turning, stopping")
                break
            current_heading = new_heading
            turns_made += 1
            
        if turns_made >= max_turns:
            print("Warning: Reached maximum number of turns without reaching desired heading")
            return
            
        # Check for blockage before moving forward
        blocked = behaviors.check_blockage()
        if blocked:
            print(f"Street at heading {current_heading} is blocked, marking as blocked")
            map.set_blocked(x, y, current_heading, True)
            return
            
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
            # After moving, check blockage
            blocked = behaviors.check_blockage()
            if blocked:
                print(f"Street at heading {h} is blocked, marking as blocked")
                map.set_blocked(x, y, h, True)
                return
        elif result == "end":
            print("Dead end detected - executing U-turn")
            original_x, original_y, original_heading = map.markdeadend()
            # Calculate new heading after U-turn
            map.calcuturn()  # This will update the heading by 180 degrees
            time.sleep(0.1)  # Add delay before turning
            handle_deadend(map, behaviors, original_x, original_y, original_heading)
            x, y, current_heading = map.pose()
            print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
            
            # After marking dead end, find and align to a valid street before continuing
            valid_headings = [h for h in range(8) 
                            if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED, STATUS.CONNECTED)
                            and not map.is_blocked(x, y, h)]
            if valid_headings:
                best_heading = min(valid_headings,
                                key=lambda h: min((h - current_heading) % 8,
                                                (current_heading - h) % 8))
                print(f"Aligning to valid street at heading {best_heading}")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                
                while current_heading != best_heading:
                    turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                    map.markturn(turn_amt, actual_angle)
                    x, y, current_heading = map.pose()
                    print(f"After alignment turn - Position: ({x}, {y}), Heading: {current_heading}")
            return
    else:
        print("No unblocked unknown streets left to explore at this intersection.")

    # Step 2: Check for UNEXPLORED streets (skip blocked)
    unexplored_headings = [h for h in range(8)
                            if current_intersection.streets[h] == STATUS.UNEXPLORED 
                            and not map.is_blocked(x, y, h)]
    print(f"\nUnblocked unexplored streets at headings: {unexplored_headings}")
    
    if unexplored_headings:
        # Find heading with smallest rotation needed
        best_heading = min(unexplored_headings,
                            key=lambda h: min((h - current_heading) % 8,
                                            (current_heading - h) % 8))
        diff = (best_heading - current_heading) % 8
        counter_diff = (current_heading - best_heading) % 8
        if diff <= counter_diff:
            turn_direction = "left"
        else:
            turn_direction = "right"
        
        print(f"Found unexplored street at heading {best_heading}, turning {turn_direction}")
        while current_heading != best_heading:
            # Check if the next heading would be blocked before turning
            next_heading = (current_heading + (1 if turn_direction == "left" else -1)) % 8
            if map.is_blocked(x, y, next_heading):
                print(f"Street at heading {next_heading} is blocked, removing from unexplored_headings")
                if next_heading in unexplored_headings:
                    unexplored_headings.remove(next_heading)
                if not unexplored_headings:
                    print("No more unblocked unexplored streets to explore")
                    return
                # Recalculate best heading
                best_heading = min(unexplored_headings,
                                key=lambda h: min((h - current_heading) % 8,
                                                (current_heading - h) % 8))
                print(f"New best heading: {best_heading}")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                continue

            turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
            map.markturn(turn_amt, actual_angle)
            x, y, new_heading = map.pose()
            # After turning, check blockage
            blocked = behaviors.check_blockage()
            if blocked:
                print(f"Street at heading {new_heading} is blocked, marking as blocked")
                map.set_blocked(x, y, new_heading, True)
                # Remove this heading from unexplored_headings
                if new_heading in unexplored_headings:
                    unexplored_headings.remove(new_heading)
                if not unexplored_headings:
                    print("No more unblocked unexplored streets to explore")
                    return
                # Recalculate best heading
                best_heading = min(unexplored_headings,
                                key=lambda h: min((h - current_heading) % 8,
                                                (current_heading - h) % 8))
                print(f"New best heading: {best_heading}")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                continue
            print(f"After turn - Position: ({x}, {y}), Heading: {new_heading}")
            # Check if we're making progress
            if new_heading == current_heading:
                print("Warning: Not making progress in turning, stopping")
                break
                
            current_heading = new_heading

        # Check for blockage before moving forward
        blocked = behaviors.check_blockage()
        if blocked:
            print(f"Street at heading {current_heading} is blocked, marking as blocked")
            map.set_blocked(x, y, current_heading, True)
            return
            
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
            # After moving, check blockage
            blocked = behaviors.check_blockage()
            if blocked:
                print(f"Street at heading {h} is blocked, marking as blocked")
                map.set_blocked(x, y, h, True)
                return
        elif result == "end":
            print("Dead end detected - executing U-turn")
            original_x, original_y, original_heading = map.markdeadend()
            # Calculate new heading after U-turn
            map.calcuturn()  # This will update the heading by 180 degrees
            time.sleep(0.1)  # Add delay before turning
            handle_deadend(map, behaviors, original_x, original_y, original_heading)
            x, y, current_heading = map.pose()
            print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
            
            # After marking dead end, find and align to a valid street before continuing
            valid_headings = [h for h in range(8) 
                            if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED, STATUS.CONNECTED)
                            and not map.is_blocked(x, y, h)]
            if valid_headings:
                best_heading = min(valid_headings,
                                key=lambda h: min((h - current_heading) % 8,
                                                (current_heading - h) % 8))
                print(f"Aligning to valid street at heading {best_heading}")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                
                while current_heading != best_heading:
                    turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                    map.markturn(turn_amt, actual_angle)
                    x, y, current_heading = map.pose()
                    print(f"After alignment turn - Position: ({x}, {y}), Heading: {current_heading}")
            return
    else:
        print("No unblocked unexplored streets left to explore at this intersection.")

    # Use Dijkstra's to find nearest intersection with unknown/unexplored streets
    found_goal = False
    min_distance = float('inf')
    best_goal = None
    
    print("\nSearching for nearest intersection with unknown/unexplored streets...")
    # First check all intersections for unknown/unexplored streets
    for tx, ty in map.intersections.keys():
        inter = map.intersections[(tx, ty)]
        unknown_or_unexplored = [h for h in range(8) 
                               if inter.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED) 
                               and not map.is_blocked(tx, ty, h)]
        if unknown_or_unexplored:
            # manhattan distance
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
        if map.goal is not None:
            map.showwithrobot()
            map.step_toward_goal(behaviors)
            x, y, current_heading = map.pose()
            print(f"Moving to goal - Position: ({x}, {y}), Heading: {current_heading}")
    else:
        print("Exploration complete! No more unknown or unexplored streets.")
        return


def handle_deadend(map, behaviors, original_x, original_y, original_heading):
    """Handle dead end by performing U-turn and line following back to intersection."""
    # Perform U-turn
    print("Executing U-turn...")
    # Turn 180 degrees
    time.sleep(0.1)  # Add delay before turning
    turn_amt, actual_angle = behaviors.turning_behavior("left")

    # Line follow back to intersection
    print("Line following back to intersection...")
    result = behaviors.follow_line()
    if result == "intersection":
        print("Back at intersection after U-turn")
        # Update position to original intersection
        map.set_pose(original_x, original_y, original_heading)
        # Mark the street as dead end now that we're back at the intersection
        inter = map.getintersection(original_x, original_y)
        if inter.streets[original_heading] != STATUS.NONEXISTENT:
            inter.streets[original_heading] = STATUS.DEADEND
    else:
        print("Warning: Did not reach intersection after U-turn")


def brain_main(io):
    drive = DriveSystem(io)
    sensor = LineSensor(io)
    angle = AngleSensor(io)
    proximity_sensor = ProximitySensor(io)
    behaviors = Behaviors(io, drive, sensor, angle, proximity_sensor)

    shared = Shared()
    map = Map()

    # Ensure the starting intersection is initialized before any map display
    map.align_to_road(behaviors)

    ui_thread = threading.Thread(target=ui, args=(shared,), daemon=True)
    ui_thread.start()

    exploring = False
    paused = False
    navigating_to_goal = False
    invalid_goal_reported = False

    try:
        while True:
            with shared.lock:
                cmd = shared.command
                goal = shared.goal
                pose = shared.pose
                shared.command = None

            if cmd == "quit":
                break
            elif cmd == "explore":
                exploring = True
                paused = False
                navigating_to_goal = False
                invalid_goal_reported = False
            elif cmd == "pause":
                paused = True
            elif cmd == "resume":
                paused = False
            elif cmd == "step":
                paused = False
                exploring = True
                navigating_to_goal = False
                invalid_goal_reported = False
            elif cmd == "goal" and goal:
                # Check if the goal intersection exists in the map
                if goal in map.intersections:
                    print(f"Setting goal to ({goal[0]}, {goal[1]})")
                    map.dijkstra(goal[0], goal[1])
                    if map.goal is not None:  # Only set navigating if dijkstra found a path
                        exploring = False
                        paused = False
                        navigating_to_goal = True
                        invalid_goal_reported = False
                        print("Path found to goal, beginning navigation...")
                    else:
                        print("No valid path found to goal.")
                        navigating_to_goal = False
                else:
                    if not invalid_goal_reported:
                        print(f"Error: Goal intersection ({goal[0]}, {goal[1]}) does not exist in the map.")
                        print("Please choose a valid intersection that has been explored.")
                        invalid_goal_reported = True
                    navigating_to_goal = False
            elif cmd in ("left", "right"):
                turn_amt, actual_angle = behaviors.turning_behavior(cmd)
                map.markturn(turn_amt, actual_angle)
                # After turning, check blockage in new heading
                x, y, h = map.pose()
                blocked = behaviors.check_blockage()
                map.set_blocked(x, y, h, blocked)
            elif cmd == "straight":
                x, y, h = map.pose()
                if map.is_blocked(x, y, h):
                    print("Cannot drive straight: street ahead is blocked!")
                    continue
                result = behaviors.follow_line()
                if result == "intersection":
                    has_street = behaviors.pull_forward()
                    map.update_connection()
                    x, y, h = map.pose()
                    inter = map.getintersection(x, y)
                    if has_street:
                        if inter.streets[h] == STATUS.UNKNOWN:
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
                    # After moving straight, check blockage in new heading
                    blocked = behaviors.check_blockage()
                    map.set_blocked(x, y, h, blocked)
                elif result == "end":
                    print("Dead end detected - executing U-turn")
                    original_x, original_y, original_heading = map.markdeadend()
                    # Calculate new heading after U-turn
                    map.calcuturn()  # This will update the heading by 180 degrees
                    time.sleep(0.1)  # Add delay before turning
                    handle_deadend(map, behaviors, original_x, original_y, original_heading)
                    x, y, current_heading = map.pose()
                    print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
            elif cmd == "save":
                name = input("Filename to save: ")
                with open(name, 'wb') as f:
                    pickle.dump(map, f)
                print("Map saved.")
            elif cmd == "load":
                name = input("Filename to load: ")
                with open(name, 'rb') as f:
                    map = pickle.load(f)
                print("Map loaded.")
            elif cmd == "pose" and pose:
                x, y, heading = pose
                if (x, y) in map.intersections:
                    map.set_pose(x, y, heading)
                    map.showwithrobot()
                else:
                    print(f"Error: Cannot set pose to ({x}, {y}) - no intersection exists at that location.")
                    print("Robot position remains unchanged.")
            elif cmd == "show":
                map.showwithrobot()

            if not paused:
                if exploring:
                    autonomous_step(map, behaviors)
                elif navigating_to_goal and map.goal is not None:
                    map.showwithrobot()
                    x, y, current_heading = map.pose()
                    current_inter = map.getintersection(x, y)
                    
                    # Check if we've reached the goal
                    if (x, y) == map.goal:
                        print("Reached goal!")
                        navigating_to_goal = False
                        map.goal = None
                        invalid_goal_reported = False
                    # Check if we have a valid path to the goal
                    elif current_inter.direction is None:
                        print("Lost path to goal, replanning...")
                        map.dijkstra(map.goal[0], map.goal[1])
                        if map.goal is None:
                            print("No valid path found to goal.")
                            navigating_to_goal = False
                    else:
                        print(f"Moving to goal - Position: ({x}, {y}), Heading: {current_heading}")
                        map.step_toward_goal(behaviors)

            if cmd == "step":
                paused = True

            map.showwithrobot()
            time.sleep(0.01)

    finally:
        drive.stop()
        io.stop()


if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Could not connect to pigpio.")
        exit()

    brain_main(io)
