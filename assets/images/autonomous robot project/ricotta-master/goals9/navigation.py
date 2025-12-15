import time
from MapBuilding import STATUS
from uithread import Shared
import math

#  for start up
def align_to_road(behaviors, map, shared):
    print("Attempting initial alignment to road...")

    x = int(input("Enter global x coordinate for current intersection: "))
    y = int(input("Enter global y coordinate for current intersection: "))
    
    # Set the initial position in the map
    map.set_pose(x, y, 0)  # Start with heading 0, will be updated later

    has_street = False
    while True:
        desired_heading = 0
        L, M, R = behaviors.sensor.read()
        if (L, M, R) == (0, 0, 0):
            print("Off-road — performing U-turn.")
            behaviors.turning_behavior("left")
            result = behaviors.follow_line()
            if result == "intersection":
                print("Back at intersection.")
                has_street = behaviors.pull_forward()
                inter = map.getintersection(map.x, map.y)
                
                print("\nEnter desired heading (0-7):")
                while True:
                    try:
                        desired_heading = int(input("Heading: "))
                        if 0 <= desired_heading <= 7:
                            break
                        else:
                            print("Please enter a number between 0 and 7")
                    except ValueError:
                        print("Please enter a valid number")
            
                # Set the heading in the map without turning the robot
                # Set the pose in the map and shared data
                map.set_pose(x, y, desired_heading)
                with shared.lock:
                    shared.robotx = x
                    shared.roboty = y
                    shared.robotheading = desired_heading
                print(f"Map heading set to: {desired_heading}")
                
                if has_street:
                    map.setstreet(map.x, map.y, desired_heading, STATUS.UNEXPLORED)
                    for delta in [-1, 1, -3, 3]:
                        diag = (desired_heading + delta) % 8
                        if inter.streets[diag] == STATUS.UNKNOWN:
                            map.setstreet(map.x, map.y, diag, STATUS.NONEXISTENT)
                else:
                    for delta in [-3, 3]:
                        map.setstreet(map.x, map.y, desired_heading, STATUS.NONEXISTENT)
                        diag = (desired_heading + delta) % 8
                        if inter.streets[diag] == STATUS.UNKNOWN:
                            map.setstreet(map.x, map.y, diag, STATUS.NONEXISTENT)
                    map.setstreet(map.x, map.y, desired_heading, STATUS.NONEXISTENT)
                    
                break
        else:
            print("On-road — following line forward.")
            # Check for blockage before moving
            blocked = behaviors.check_blockage(desired_heading)
            if blocked:
                print("Street ahead is blocked, marking as blocked")
                map.set_blocked(map.x, map.y, map.heading, True)
                # Get current intersection
                inter = map.getintersection(map.x, map.y)
                # Find and align to a valid unblocked street
                valid_headings = [h for h in range(8) 
                                if inter.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED, STATUS.CONNECTED)
                                and not map.is_blocked(map.x, map.y, h)]
                if valid_headings:
                    best_heading = min(valid_headings,
                                    key=lambda h: min((h - map.heading) % 8,
                                                    (map.heading - h) % 8))
                    print(f"Aligning to valid street at heading {best_heading}")
                    diff = (best_heading - map.heading) % 8
                    counter_diff = (map.heading - best_heading) % 8
                    turn_direction = "left" if diff <= counter_diff else "right"
                    
                    while map.heading != best_heading:
                        turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                        map.markturn(turn_amt, actual_angle)
                        print(f"After alignment turn - Position: ({map.x}, {map.y}), Heading: {map.heading}")
                else:
                    print("No valid unblocked streets available")
                    return
                continue

            result = behaviors.follow_line()
            if result == "intersection":
                print("Reached first intersection.")
                has_street = behaviors.pull_forward()
                inter = map.getintersection(map.x, map.y)
                
                print("\nEnter desired heading (0-7):")
                while True:
                    try:
                        desired_heading = int(input("Heading: "))
                        if 0 <= desired_heading <= 7:
                            break
                        else:
                            print("Please enter a number between 0 and 7")
                    except ValueError:
                        print("Please enter a valid number")
            
                # Set the heading in the map without turning the robot
                map.set_pose(map.x, map.y, desired_heading)
                print(f"Map heading set to: {desired_heading}")
                
                if has_street:
                    map.setstreet(map.x, map.y, desired_heading, STATUS.UNEXPLORED)
                    for delta in [-1, 1, -3, 3]:
                        diag = (desired_heading + delta) % 8
                        if inter.streets[diag] == STATUS.UNKNOWN:
                            map.setstreet(map.x, map.y, diag, STATUS.NONEXISTENT)
                else:
                    map.setstreet(map.x, map.y, desired_heading, STATUS.NONEXISTENT)
                    for delta in [-3, 3]:
                        diag = (desired_heading + delta) % 8
                        if inter.streets[diag] == STATUS.UNKNOWN:
                            map.setstreet(map.x, map.y, diag, STATUS.NONEXISTENT)
                break
            elif result == "end":
                print("Dead end detected - executing U-turn")
                original_x, original_y, original_heading = map.markdeadend()
                # Calculate new heading after U-turn
                time.sleep(0.1)  # Add delay before turning
                handle_deadend(map, behaviors, original_x, original_y, original_heading)
                x, y, current_heading = map.pose()
                print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
        break


def step_toward_goal(map, behaviors):
    x, y, h = map.pose()
    inter = map.getintersection(x, y)

    # If the robot is at the goal, clear the goal and return
    if (x, y) == map.goal:
        print("Goal reached!")
        map.cleargoal()
        return

    # If there are no known paths to goal, replan
    direction = inter.direction
    if direction is None:
        print("No known path to goal, replanning...")
        map.dijkstra(map.goal[0], map.goal[1])
        inter = map.getintersection(x, y)
        direction = inter.direction
        if direction is None:
            print("No possible route to goal - goal is unreachable!")
            map.clear_blockages()  # Clear blockages so robot can recover
            print("Blockages cleared, retrying path to goal...")
            map.dijkstra(map.goal[0], map.goal[1])
            step_toward_goal(map, behaviors)
            return

    # If the robot is facing the goal, move forward
    if direction != h:
        # Decide turn direction
        turn_amt = (direction - h) % 8
        if turn_amt > 4:
            turn_amt -= 8
        turn_dir = "left" if turn_amt > 0 else "right"

        # Start turning, one step (45) at a time, until at correct heading
        turns_made = 0
        max_turns = 4
        while h != direction and turns_made < max_turns:
            print(f"Current heading: {h}, desired: {direction}")

            # Turn one 45 step
            step = 1 if turn_dir == "left" else -1
            turn_result = behaviors.turning_behavior(turn_dir)  # should return number of steps taken (±1)
            turn_amount, actual_angle = turn_result
            map.markturn(turn_amount, actual_angle)
            turns_made += 1
            time.sleep(0.1)

            # Read sensors after turn
            L, M, R = behaviors.sensor.read()
            is_on_line = (L, M, R) != (0, 0, 0)

            # Check for a valid street at new heading using sensor AND map
            inter = map.getintersection(map.x, map.y)

            # Check for blockage after turn
            blocked = behaviors.check_blockage(h)
            if blocked:
                print("Blocked street detected! Replanning path...")
                # Mark the current street as blocked using current position and heading
                map.set_blocked(x, y, map.heading, True)
                # Replan path to goal from current position
                map.dijkstra(map.goal[0], map.goal[1])
                # Check if a valid path was found after replanning
                inter = map.getintersection(x, y)
                if inter.direction is None:
                    print("No possible route to replan to - goal is unreachable!")
                    map.cleargoal()
                    return
                # Get new direction after replanning
                direction = inter.direction
                # If we need to turn to follow new path
                if direction != map.heading:
                    print(f"Turning to follow new path. Current heading: {map.heading}, New direction: {direction}")
                    turn_amt = (direction - map.heading) % 8
                    if turn_amt > 4:
                        turn_amt -= 8
                    turn_dir = "left" if turn_amt > 0 else "right"
                    turns_made = 0
                    max_turns = 4
                    while map.heading != direction and turns_made < max_turns:
                        turn_result = behaviors.turning_behavior(turn_dir)
                        turn_amount, actual_angle = turn_result
                        map.markturn(turn_amount, actual_angle)
                        x, y, h = map.pose()
                        turns_made += 1
                        time.sleep(0.1)
                    if turns_made >= max_turns:
                        print("Warning: Reached maximum number of turns without reaching desired heading")
                        return
                return

            print(f"Sensor sees line: {is_on_line}, map says street at heading {map.heading} is {inter.streets[map.heading].name}")
            valid_street = inter.streets[map.heading] not in (STATUS.NONEXISTENT, STATUS.DEADEND)

            if map.heading == direction and is_on_line and valid_street:
                print("Aligned to correct heading with valid street — done turning.")
                break
            elif is_on_line:
                print("At wrong street. Pausing briefly before continuing turn...")
                behaviors.drive.stop()
                time.sleep(0.3)  # Let robot sit on wrong street before resuming

        if turns_made >= max_turns:
            print("Warning: Reached maximum number of turns without reaching desired heading")
            # Force set the heading to prevent further turning
            map.set_pose(map.x, map.y, direction)
            return
        # After turning, update h
        x, y, h = map.pose()

    # Now, robot is facing the correct direction
    print("Advancing toward goal...")
    blocked = behaviors.check_blockage(h)
    if blocked:
        print(f"Blocked street detected at heading {h}! Current position: ({x}, {y})")
        # Mark the current street as blocked using current position and heading
        map.set_blocked(x, y, h, True)
        print(f"Marked street at heading {h} as blocked")
        # Replan path to goal from current position
        map.dijkstra(map.goal[0], map.goal[1])
        # Check if a valid path was found after replanning
        inter = map.getintersection(x, y)
        if inter.direction is None:
            print("No possible route to replan to - goal is unreachable!")
            map.clear_blockages()  # Clear blockages so robot can recover
            print("Blockages cleared, retrying path to goal...")
            map.dijkstra(map.goal[0], map.goal[1])
            step_toward_goal(map, behaviors)
            return
        # Get new direction after replanning
        direction = inter.direction
        print(f"New direction after replanning: {direction}")
        # If we need to turn to follow new path
        if direction != h:
            print(f"Turning to follow new path. Current heading: {h}, New direction: {direction}")
            turn_amt = (direction - h) % 8
            if turn_amt > 4:
                turn_amt -= 8
            turn_dir = "left" if turn_amt > 0 else "right"
            print(f"Need to turn {turn_amt} steps {turn_dir}")
            turns_made = 0
            max_turns = 4
            while h != direction and turns_made < max_turns:
                print(f"Making turn {turns_made + 1}/{max_turns}")
                turn_result = behaviors.turning_behavior(turn_dir)
                turn_amount, actual_angle = turn_result
                print(f"Turn result: amount={turn_amount}, actual_angle={actual_angle}")
                map.markturn(turn_amount, actual_angle)
                x, y, h = map.pose()
                print(f"After turn - Position: ({x}, {y}), Heading: {h}")
                turns_made += 1
                time.sleep(0.1)
            if turns_made >= max_turns:
                print("Warning: Reached maximum number of turns without reaching desired heading")
                return
        return

    # Actually move forward toward the goal
    result = behaviors.follow_line()
    if result == "intersection":
        has_street = behaviors.pull_forward()
        if has_street:
            map.setstreet(x, y, h, STATUS.UNEXPLORED)
            for delta in [-1, 1, 3, -3]:
                side_heading = (map.heading + delta) % 8
                if inter.streets[side_heading] == STATUS.UNKNOWN:
                    inter.streets[side_heading] = STATUS.NONEXISTENT
        else:
            map.setstreet(x, y, h, STATUS.NONEXISTENT)
            for delta in [-3, 3]:
                side_heading = (map.heading + delta) % 8
                if inter.streets[side_heading] == STATUS.UNKNOWN:
                    inter.streets[side_heading] = STATUS.NONEXISTENT
        map.update_connection()
        # Replan at each intersection to ensure optimal path
        map.dijkstra(map.goal[0], map.goal[1])
    elif result == "end":
        print("Dead end detected - executing U-turn")
        original_x, original_y, original_heading = map.markdeadend()
        # Calculate new heading after U-turn
        time.sleep(0.1)  # Add delay before turning
        handle_deadend(map, behaviors, original_x, original_y, original_heading)
        x, y, current_heading = map.pose()
        print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")


def handle_deadend(map, behaviors, original_x, original_y, original_heading):
    """Handle dead end by performing U-turn and line following back to intersection."""
    # Perform U-turn
    print("Executing U-turn...")
    turn_amt, actual_angle = behaviors.turning_behavior("left")
    map.calcuturn()  # Update heading in the map by 4 (180 degrees)
    
    # Line follow back to intersection
    print("Line following back to intersection...")
    result = behaviors.follow_line()
    if result == "intersection":
        print("Back at intersection after U-turn")
        has_street = behaviors.pull_forward()
        # After pulling forward, we should be at the intersection facing away from the dead end
        x, y, current_heading = map.pose()
        # Mark the street as dead end now that we're back at the intersection
        # Note: We mark the street in the opposite direction of our current heading
        inter = map.getintersection(x, y)
        deadend_heading = (current_heading + 4) % 8  # Opposite of current heading
        if inter.streets[deadend_heading] != STATUS.NONEXISTENT:
            inter.streets[deadend_heading] = STATUS.DEADEND
            print(f"Marked street at heading {deadend_heading} as DEADEND")
        
        # Mark the street we came from based on pull_forward result
        if has_street:
            if inter.streets[current_heading] == STATUS.UNKNOWN:
                inter.streets[current_heading] = STATUS.UNEXPLORED
                print(f"Marked street at heading {current_heading} as UNEXPLORED")
            # Mark diagonals as NONEXISTENT
            for delta in [-1, 1, -3, 3]:
                diag_heading = (current_heading + delta) % 8
                if inter.streets[diag_heading] == STATUS.UNKNOWN:
                    inter.streets[diag_heading] = STATUS.NONEXISTENT
                    print(f"Marked diagonal street at heading {diag_heading} as NONEXISTENT")
        else:
            # If no street ahead, mark current heading as NONEXISTENT
            inter.streets[current_heading] = STATUS.NONEXISTENT
            print(f"Marked street at heading {current_heading} as NONEXISTENT")
            # Mark back diagonals as NONEXISTENT
            for delta in [-3, 3]:
                diag_heading = (current_heading + delta) % 8
                if inter.streets[diag_heading] == STATUS.UNKNOWN:
                    inter.streets[diag_heading] = STATUS.NONEXISTENT
                    print(f"Marked back diagonal street at heading {diag_heading} as NONEXISTENT")
        
        # Debug print: robot's position, heading, and next intersection
        print(f"[DEBUG] After dead end handling: Position: ({x}, {y}), Heading: {current_heading}")
        # Find next intersection to go to (if any)
        unexplored = []
        for (ix, iy), intersection in map.intersections.items():
            for heading in range(8):
                if intersection.streets[heading] in (STATUS.UNKNOWN, STATUS.UNEXPLORED):
                    unexplored.append((ix, iy))
                    break
        if unexplored:
            # Find the closest unexplored intersection
            next_goal = min(unexplored, key=lambda pos: abs(pos[0] - x) + abs(pos[1] - y))
            print(f"[DEBUG] Next intersection to go to: {next_goal}")
        else:
            print("[DEBUG] No more unexplored intersections left.")
    else:
        print("Warning: Did not reach intersection after U-turn")

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

    # Check if we need to update street status for current heading
    if current_intersection.streets[current_heading] == STATUS.UNKNOWN:
        # Check if we're on a line (which would indicate a street exists)
        L, M, R = behaviors.sensor.read()
        has_street = (L, M, R) != (0, 0, 0)
        print(f"Street ahead exists: {has_street}")
        
        if has_street:
            # Mark current street as UNEXPLORED
            map.setstreet(x, y, current_heading, STATUS.UNEXPLORED)
            print(f"Marked street at heading {current_heading} as UNEXPLORED")
            # Mark diagonals as NONEXISTENT
            for delta in [-1, 1, -3, 3]:
                diag_heading = (current_heading + delta) % 8
                if current_intersection.streets[diag_heading] == STATUS.UNKNOWN:
                    map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                    print(f"Marked diagonal street at heading {diag_heading} as NONEXISTENT")
        else:
            # If no street ahead, mark current heading as NONEXISTENT
            map.setstreet(x, y, current_heading, STATUS.NONEXISTENT)
            print(f"Marked street at heading {current_heading} as NONEXISTENT")
            # Mark back diagonals as NONEXISTENT
            for delta in [-3, 3]:
                diag_heading = (current_heading + delta) % 8
                if current_intersection.streets[diag_heading] == STATUS.UNKNOWN:
                    map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                    print(f"Marked back diagonal street at heading {diag_heading} as NONEXISTENT")

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
            best_heading = min(unknown_headings, key=lambda h: min((h - current_heading) % 8, (current_heading - h) % 8))
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
            turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
            map.markturn(turn_amt, actual_angle)
            x, y, current_heading = map.pose()
            
            print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")
            
            # Check if we're making progress towards the desired heading
            current_diff = (best_heading - current_heading) % 8
            prev_diff = (best_heading - prev_heading) % 8
            if current_diff >= prev_diff:
                print("Warning: Not making progress towards desired heading, stopping")
                break
                
            prev_heading = current_heading
            turns_made += 1
            
            # Add a small delay between turns to prevent rapid spinning
            time.sleep(0.1)
            
            
        # Check for blockage before moving forward
        blocked = behaviors.check_blockage(current_heading)
        if blocked:
            print(f"Street at heading {current_heading} is blocked, marking as blocked")
            map.set_blocked(x, y, current_heading, True)
            # Find next valid heading to explore
            valid_headings = [h for h in range(8) 
                            if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED)
                            and not map.is_blocked(x, y, h)
                            and current_intersection.streets[h] != STATUS.DEADEND]
            if valid_headings:
                best_heading = min(valid_headings,
                                key=lambda h: min((h - current_heading) % 8,
                                                (current_heading - h) % 8))
                print(f"Turning to next valid heading {best_heading}")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                
                turns_made = 0
                max_turns = 4
                while current_heading != best_heading and turns_made < max_turns:
                    turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                    map.markturn(turn_amt, actual_angle)
                    x, y, current_heading = map.pose()
                    print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")
                    turns_made += 1
                if turns_made >= max_turns:
                    print("Warning: Reached maximum number of turns without reaching desired heading; keeping current heading.")
                    return
            else:
                print("No more valid headings to explore at this intersection")
                return
        else:
            result = behaviors.follow_line()
            if result == "intersection":
                has_street = behaviors.pull_forward()
                map.update_connection()
                x, y, h = map.pose()
                print(f"Reached intersection - Position: ({x}, {y}), Heading: {h}")
                print(f"Street ahead exists: {has_street}")
                
                # Check for blockage at the intersection
                blocked = behaviors.check_blockage(h)
                if blocked:
                    print(f"Street at heading {h} is blocked, marking as blocked")
                    map.set_blocked(x, y, h, True)
                    # Mark diagonals as NONEXISTENT if they are UNKNOWN
                    if has_street:
                        inter = map.getintersection(x, y)
                        for delta in [-1, 1, -3, 3]:
                            diag = (h + delta) % 8
                            if inter.streets[diag] == STATUS.UNKNOWN:
                                map.setstreet(x, y, diag, STATUS.NONEXISTENT)
                    else:
                        inter = map.getintersection(x, y)
                        for delta in [-3, 3]:
                            diag = (h + delta) % 8
                            if inter.streets[diag] == STATUS.UNKNOWN:
                                map.setstreet(x, y, diag, STATUS.NONEXISTENT)
                    # Find next valid heading to explore
                    valid_headings = [h for h in range(8) 
                                    if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED)
                                    and not map.is_blocked(x, y, h)
                                    and current_intersection.streets[h] != STATUS.DEADEND]
                    if valid_headings:
                        best_heading = min(valid_headings,
                                        key=lambda h: min((h - current_heading) % 8,
                                                        (current_heading - h) % 8))
                        print(f"Turning to next valid heading {best_heading}")
                        diff = (best_heading - current_heading) % 8
                        counter_diff = (current_heading - best_heading) % 8
                        turn_direction = "left" if diff <= counter_diff else "right"
                        
                        turns_made = 0
                        max_turns = 4
                        while current_heading != best_heading and turns_made < max_turns:
                            turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                            map.markturn(turn_amt, actual_angle)
                            x, y, current_heading = map.pose()
                            print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")
                            turns_made += 1
                        if turns_made >= max_turns:
                            print("Warning: Reached maximum number of turns without reaching desired heading; keeping current heading.")
                            return
                    else:
                        print("No more valid headings to explore at this intersection")
                        return
                
                inter = map.intersections.get((x, y))
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
            if result == "end":
                print("Dead end detected - executing U-turn")
                original_x, original_y, original_heading = map.markdeadend()
                # Calculate new heading after U-turn
                handle_deadend(map, behaviors, original_x, original_y, original_heading)
                x, y, current_heading = map.pose()
                print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
                
                # After marking dead end, find and align to a valid street before continuing
                valid_headings = [h for h in range(8) 
                                if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED, STATUS.CONNECTED)
                                and not map.is_blocked(x, y, h)
                                and current_intersection.streets[h] != STATUS.DEADEND]
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
        prev_heading = current_heading  # Initialize prev_heading
        max_turns = 4  # Maximum number of turns to prevent infinite loop
        turns_made = 0  # Initialize turns_made
        
        while current_heading != best_heading and turns_made < max_turns:
            turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
            map.markturn(turn_amt, actual_angle)
            x, y, current_heading = map.pose()
            
            print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")
            
            # Check if we're making progress towards the desired heading
            current_diff = (best_heading - current_heading) % 8
            prev_diff = (best_heading - prev_heading) % 8
            if current_diff >= prev_diff:
                print("Warning: Not making progress towards desired heading, stopping")
                break
                
            prev_heading = current_heading
            turns_made += 1
            
            # Add a small delay between turns to prevent rapid spinning
            time.sleep(0.1)

        if turns_made >= max_turns:
            print("Warning: Reached maximum number of turns without reaching desired heading; keeping current heading.")
            return

        # Check for blockage before moving forward
        blocked = behaviors.check_blockage(current_heading)
        if blocked:
            print(f"Street at heading {current_heading} is blocked, marking as blocked")
            map.set_blocked(x, y, current_heading, True)
            # Find next valid heading to explore
            valid_headings = [h for h in range(8) 
                            if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED)
                            and not map.is_blocked(x, y, h)
                            and current_intersection.streets[h] != STATUS.DEADEND]
            if valid_headings:
                best_heading = min(valid_headings,
                                key=lambda h: min((h - current_heading) % 8,
                                                (current_heading - h) % 8))
                print(f"Turning to next valid heading {best_heading}")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                
                turns_made = 0
                max_turns = 4
                while current_heading != best_heading and turns_made < max_turns:
                    turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                    map.markturn(turn_amt, actual_angle)
                    x, y, current_heading = map.pose()
                    print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")
                    turns_made += 1
                if turns_made >= max_turns:
                    print("Warning: Reached maximum number of turns without reaching desired heading; keeping current heading.")
                    return
            else:
                print("No more valid headings to explore at this intersection")
                return
        else:
            result = behaviors.follow_line()
            if result == "intersection":
                has_street = behaviors.pull_forward()
                map.update_connection()
                x, y, h = map.pose()
                print(f"Reached intersection - Position: ({x}, {y}), Heading: {h}")
                print(f"Street ahead exists: {has_street}")
                
                # Check for blockage at the intersection
                blocked = behaviors.check_blockage(h)
                if blocked:
                    print(f"Street at heading {h} is blocked, marking as blocked")
                    map.set_blocked(x, y, h, True)
                    # Mark diagonals as NONEXISTENT if they are UNKNOWN
                    if has_street:
                        inter = map.getintersection(x, y)
                        for delta in [-1, 1, -3, 3]:
                            diag = (h + delta) % 8
                            if inter.streets[diag] == STATUS.UNKNOWN:
                                map.setstreet(x, y, diag, STATUS.NONEXISTENT)
                    else:
                        inter = map.getintersection(x, y)
                        for delta in [-3, 3]:
                            diag = (h + delta) % 8
                            if inter.streets[diag] == STATUS.UNKNOWN:
                                map.setstreet(x, y, diag, STATUS.NONEXISTENT)

                    # Find next valid heading to explore
                    valid_headings = [h for h in range(8) 
                                    if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED)
                                    and not map.is_blocked(x, y, h)
                                    and current_intersection.streets[h] != STATUS.DEADEND]
                    if valid_headings:
                        best_heading = min(valid_headings,
                                        key=lambda h: min((h - current_heading) % 8,
                                                        (current_heading - h) % 8))
                        print(f"Turning to next valid heading {best_heading}")
                        diff = (best_heading - current_heading) % 8
                        counter_diff = (current_heading - best_heading) % 8
                        turn_direction = "left" if diff <= counter_diff else "right"
                        
                        turns_made = 0
                        max_turns = 4
                        while current_heading != best_heading and turns_made < max_turns:
                            turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                            map.markturn(turn_amt, actual_angle)
                            x, y, current_heading = map.pose()
                            print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")
                            turns_made += 1
                        if turns_made >= max_turns:
                            print("Warning: Reached maximum number of turns without reaching desired heading; keeping current heading.")
                            return
                    else:
                        print("No more valid headings to explore at this intersection")
                        return
                
                inter = map.intersections.get((x, y))
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
            if result == "end":
                print("Dead end detected - executing U-turn")
                original_x, original_y, original_heading = map.markdeadend()
                # Calculate new heading after U-turn
                handle_deadend(map, behaviors, original_x, original_y, original_heading)
                x, y, current_heading = map.pose()
                print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
                
                # After marking dead end, find and align to a valid street before continuing
                valid_headings = [h for h in range(8) 
                                if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED, STATUS.CONNECTED)
                                and not map.is_blocked(x, y, h)
                                and current_intersection.streets[h] != STATUS.DEADEND]
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
            step_toward_goal(map, behaviors)
            x, y, current_heading = map.pose()
            print(f"Moving to goal - Position: ({x}, {y}), Heading: {current_heading}")
    else:
        print("Exploration complete! No more unknown or unexplored streets.")
        return


def get_heading_toward_goal(current_pos, goal_pos):
    """Calculate the heading that points toward the goal from current position."""
    dx = goal_pos[0] - current_pos[0]
    dy = goal_pos[1] - current_pos[1]
    
    # Calculate angle in radians
    angle = math.atan2(dy, dx)
    
    # Convert to heading (0-7, where 0 is East, 2 is South, 4 is West, 6 is North)
    heading = int(round(angle * 4 / math.pi)) % 8
    
    return heading

def is_heading_toward_goal(current_pos, current_heading, goal_pos):
    """Check if the current heading is taking us toward the goal."""
    # Get the ideal heading toward goal
    ideal_heading = get_heading_toward_goal(current_pos, goal_pos)
    
    # Calculate how far we are from the ideal heading
    heading_diff = min((current_heading - ideal_heading) % 8,
                      (ideal_heading - current_heading) % 8)
    
    # If we're within 1 step of the ideal heading, we're heading toward the goal
    return heading_diff <= 1

def directed_exploration(map, behaviors, goal):
    """
    Directed exploration: like autonomous_step, but when using Dijkstra to pick the next intersection,
    use the advanced logic: for each interesting intersection, sum the Dijkstra cost from the robot to the intersection
    and the Euclidean distance from that intersection to the ultimate goal. Pick the intersection with the lowest total cost as the next goal.
    """
    print("Directed exploration mode: Robot is exploring the map toward a yet-to-be-discovered goal...")
    x, y, current_heading = map.pose()
    print(f"\nCurrent Position: ({x}, {y}), Heading: {current_heading}")
    current_intersection = map.intersections.get((x, y))

    # Check if we've reached the goal intersection
    if (x, y) == goal:
        print("Goal intersection reached! Stopping exploration.")
        inter = map.intersections.get((x, y))
        if inter:
            if inter.streets[current_heading] == STATUS.UNKNOWN:
                map.setstreet(x, y, current_heading, STATUS.UNEXPLORED)
            # Mark diagonals as NONEXISTENT
            for delta in [-1, 1, -3, 3]:
                diag_heading = (current_heading + delta) % 8
                if inter.streets[diag_heading] == STATUS.UNKNOWN:
                    map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                    print(f"Marked diagonal street at heading {diag_heading} as NONEXISTENT")
        return

    if not current_intersection:
        print("Error: Robot not at a valid intersection")
        return

    # Print current intersection state
    print("\nCurrent intersection street states:")
    for h in range(8):
        status = current_intersection.streets[h]
        blocked = current_intersection.blocked[h] if hasattr(current_intersection, 'blocked') else False
        print(f"Heading {h}: {status} {'(BLOCKED)' if blocked else ''}")

    # Function to calculate where we would end up if we went in a given heading
    def get_potential_position(heading):
        # Calculate the next intersection position based on heading
        # Headings: 0=North, 1=Northwest, 2=West, 3=Southwest, 4=South, 5=Southeast, 6=East, 7=Northeast
        dx = [0, -1, -1, -1, 0, 1, 1, 1]  # x changes for each heading
        dy = [1, 1, 0, -1, -1, -1, 0, 1]  # y changes for each heading
        return (x + dx[heading], y + dy[heading])

    # Step 1: Check for UNKNOWN streets (skip blocked)
    unknown_and_unexplored_headings = [h for h in range(8)
                        if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED)
                        and not map.is_blocked(x, y, h)]
    print(f"\nUnblocked unknown streets at headings: {unknown_and_unexplored_headings}")

    if unknown_and_unexplored_headings:
        # Calculate potential positions and their distances to goal for each unknown heading
        heading_distances = []
        for heading in unknown_and_unexplored_headings:
            potential_pos = get_potential_position(heading)
            distance = math.sqrt((potential_pos[0] - goal[0])**2 + (potential_pos[1] - goal[1])**2)
            heading_distances.append((heading, distance))
        
        # Choose the heading that leads to the position closest to the goal
        best_heading, best_distance = min(heading_distances, key=lambda x: x[1])
        print(f"[DEBUG] Found unknown street leading closest to goal - Best heading: {best_heading}, Distance: {best_distance}")
        
        # Skip turning if we're already at the best heading
        if best_heading == current_heading:
            print("[DEBUG] Already at best heading, no need to turn")
        else:
            diff = (best_heading - current_heading) % 8
            counter_diff = (current_heading - best_heading) % 8
            turn_direction = "left" if diff <= counter_diff else "right"
            print(f"[DEBUG] Turning {turn_direction} from {current_heading} to {best_heading}")
            
            # Rest of the unknown streets handling code remains the same
            prev_heading = current_heading
            max_turns = 4
            turns_made = 0
            while current_heading != best_heading and turns_made < max_turns:
                turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                map.markturn(turn_amt, actual_angle)
                x, y, current_heading = map.pose()
                current_diff = (best_heading - current_heading) % 8
                prev_diff = (best_heading - prev_heading) % 8
                if current_diff >= prev_diff:
                    print("Warning: Not making progress towards desired heading, stopping")
                    break
                prev_heading = current_heading
                turns_made += 1
                time.sleep(0.1)
            if turns_made >= max_turns:
                print("Warning: Reached maximum number of turns without reaching desired heading")
                map.set_pose(x, y, best_heading)
                return
        blocked = behaviors.check_blockage(current_heading)
        if blocked:
            print(f"Street at heading {current_heading} is blocked, marking as blocked")
            map.set_blocked(x, y, current_heading, True)
            # Find next valid heading to explore, using same distance-based logic
            valid_headings = [h for h in range(8) 
                            if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED)
                            and not map.is_blocked(x, y, h)
                            and current_intersection.streets[h] != STATUS.DEADEND]
            if valid_headings:
                # Calculate distances for valid headings
                heading_distances = []
                for heading in valid_headings:
                    potential_pos = get_potential_position(heading)
                    distance = math.sqrt((potential_pos[0] - goal[0])**2 + (potential_pos[1] - goal[1])**2)
                    heading_distances.append((heading, distance))
                best_heading, best_distance = min(heading_distances, key=lambda x: x[1])
                print(f"Turning to next valid heading {best_heading} (closest to goal, distance: {best_distance})")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                
                while current_heading != best_heading:
                    turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                    map.markturn(turn_amt, actual_angle)
                    x, y, current_heading = map.pose()
                    print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")
                    
            else:
                print("No more valid headings to explore at this intersection")
                return
        result = behaviors.follow_line()
        if result == "intersection":
            has_street = behaviors.pull_forward()
            map.update_connection()
            x, y, h = map.pose()
            print(f"Reached intersection - Position: ({x}, {y}), Heading: {h}")
            
            # Update street status for all intersections, not just the goal
            inter = map.intersections.get((x, y))
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
            
            # Check if we've reached the goal intersection
            if (x, y) == goal:
                inter = map.intersections.get((x, y))
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

            blocked = behaviors.check_blockage(h)
            print("Goal intersection reached! Stopping exploration.")  
            if blocked:
                print(f"Street at heading {h} is blocked, marking as blocked")
                map.set_blocked(x, y, h, True)
                return

        elif result == "end":
            print("Dead end detected - executing U-turn")
            original_x, original_y, original_heading = map.markdeadend()
            # Calculate new heading after U-turn
            handle_deadend(map, behaviors, original_x, original_y, original_heading)
            x, y, current_heading = map.pose()
            print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
            # Find next valid heading to explore using distance-based logic
            valid_headings = [h for h in range(8) 
                            if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED)
                            and not map.is_blocked(x, y, h)
                            and current_intersection.streets[h] != STATUS.DEADEND]
            if valid_headings:
                # Calculate distances for valid headings
                heading_distances = []
                for heading in valid_headings:
                    potential_pos = get_potential_position(heading)
                    distance = math.sqrt((potential_pos[0] - goal[0])**2 + (potential_pos[1] - goal[1])**2)
                    heading_distances.append((heading, distance))
                best_heading, best_distance = min(heading_distances, key=lambda x: x[1])
                print(f"Turning to next valid heading {best_heading} (closest to goal, distance: {best_distance})")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                
                while current_heading != best_heading:
                    turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                    map.markturn(turn_amt, actual_angle)
                    x, y, current_heading = map.pose()
                    print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")
            else:
                print("No more valid headings to explore at this intersection")
                return
            
            # After marking dead end, find and align to a valid street before continuing
            valid_headings = [h for h in range(8) 
                            if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED, STATUS.CONNECTED)
                            and not map.is_blocked(x, y, h)
                            and current_intersection.streets[h] != STATUS.DEADEND]
            if valid_headings:
                # Calculate distances for valid headings
                heading_distances = []
                for heading in valid_headings:
                    potential_pos = get_potential_position(heading)
                    distance = math.sqrt((potential_pos[0] - goal[0])**2 + (potential_pos[1] - goal[1])**2)
                    heading_distances.append((heading, distance))
                best_heading, best_distance = min(heading_distances, key=lambda x: x[1])
                print(f"Aligning to valid street at heading {best_heading} (closest to goal, distance: {best_distance})")
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
        # Calculate potential positions and their distances to goal for each unexplored heading
        heading_distances = []
        for heading in unexplored_headings:
            potential_pos = get_potential_position(heading)
            distance = math.sqrt((potential_pos[0] - goal[0])**2 + (potential_pos[1] - goal[1])**2)
            heading_distances.append((heading, distance))
        
        # Choose the heading that leads to the position closest to the goal
        best_heading, best_distance = min(heading_distances, key=lambda x: x[1])
        print(f"[DEBUG] Found unexplored street leading closest to goal - Best heading: {best_heading}, Distance: {best_distance}")
        
        # Skip turning if we're already at the best heading
        if best_heading == current_heading:
            print("[DEBUG] Already at best heading, no need to turn")
        else:
            diff = (best_heading - current_heading) % 8
            counter_diff = (current_heading - best_heading) % 8
            turn_direction = "left" if diff <= counter_diff else "right"
            print(f"[DEBUG] Turning {turn_direction} from {current_heading} to {best_heading}")
            
            # Rest of the unexplored streets handling code remains the same
            prev_heading = current_heading
            max_turns = 4
            turns_made = 0
            while current_heading != best_heading and turns_made < max_turns:
                turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                map.markturn(turn_amt, actual_angle)
                x, y, current_heading = map.pose()
                current_diff = (best_heading - current_heading) % 8
                prev_diff = (best_heading - prev_heading) % 8
                if current_diff >= prev_diff:
                    print("Warning: Not making progress towards desired heading, stopping")
                    break
                prev_heading = current_heading
                turns_made += 1
                time.sleep(0.1)
            if turns_made >= max_turns:
                print("Warning: Reached maximum number of turns without reaching desired heading")
                map.set_pose(x, y, best_heading)
                return
        blocked = behaviors.check_blockage(current_heading)
        if blocked:
            print(f"Street at heading {current_heading} is blocked, marking as blocked")
            map.set_blocked(x, y, current_heading, True)
            # Find next valid heading to explore, using same distance-based logic
            valid_headings = [h for h in range(8) 
                            if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED)
                            and not map.is_blocked(x, y, h)
                            and current_intersection.streets[h] != STATUS.DEADEND]
            if valid_headings:
                # Calculate distances for valid headings
                heading_distances = []
                for heading in valid_headings:
                    potential_pos = get_potential_position(heading)
                    distance = math.sqrt((potential_pos[0] - goal[0])**2 + (potential_pos[1] - goal[1])**2)
                    heading_distances.append((heading, distance))
                best_heading, best_distance = min(heading_distances, key=lambda x: x[1])
                print(f"Turning to next valid heading {best_heading} (closest to goal, distance: {best_distance})")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                
                while current_heading != best_heading:
                    turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                    map.markturn(turn_amt, actual_angle)
                    x, y, current_heading = map.pose()
                    print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")
            else:
                print("No more valid headings to explore at this intersection")
                return
        result = behaviors.follow_line()
        if result == "intersection":
            has_street = behaviors.pull_forward()
            map.update_connection()
            x, y, h = map.pose()
            print(f"Reached intersection - Position: ({x}, {y}), Heading: {h}")
            
            # Update street status for all intersections, not just the goal
            inter = map.intersections.get((x, y))
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
            
            # Check if we've reached the goal intersection
            if (x, y) == goal:
                print("Goal intersection reached! Stopping exploration.")
                return
                
            inter = map.intersections.get((x, y))
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
            blocked = behaviors.check_blockage(h)
            if blocked:
                print(f"Street at heading {h} is blocked, marking as blocked")
                map.set_blocked(x, y, h, True)
                return
        elif result == "end":
            print("Dead end detected - executing U-turn")
            original_x, original_y, original_heading = map.markdeadend()
            # Calculate new heading after U-turn
            handle_deadend(map, behaviors, original_x, original_y, original_heading)
            x, y, current_heading = map.pose()
            print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
            # Find next valid heading to explore using distance-based logic
            valid_headings = [h for h in range(8) 
                            if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED)
                            and not map.is_blocked(x, y, h)
                            and current_intersection.streets[h] != STATUS.DEADEND]
            if valid_headings:
                # Calculate distances for valid headings
                heading_distances = []
                for heading in valid_headings:
                    potential_pos = get_potential_position(heading)
                    distance = math.sqrt((potential_pos[0] - goal[0])**2 + (potential_pos[1] - goal[1])**2)
                    heading_distances.append((heading, distance))
                best_heading, best_distance = min(heading_distances, key=lambda x: x[1])
                print(f"Turning to next valid heading {best_heading} (closest to goal, distance: {best_distance})")
                diff = (best_heading - current_heading) % 8
                counter_diff = (current_heading - best_heading) % 8
                turn_direction = "left" if diff <= counter_diff else "right"
                
                while current_heading != best_heading:
                    turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                    map.markturn(turn_amt, actual_angle)
                    x, y, current_heading = map.pose()
                    print(f"After turn - Position: ({x}, {y}), Heading: {current_heading}")
            else:
                print("No more valid headings to explore at this intersection")
                return
            print("No blockage detected - executing U-turn")
            original_x, original_y, original_heading = map.markdeadend()
            time.sleep(0.1)
            handle_deadend(map, behaviors, original_x, original_y, original_heading)
            x, y, current_heading = map.pose()
            print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
            
            # After marking dead end, find and align to a valid street before continuing
            valid_headings = [h for h in range(8) 
                            if current_intersection.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED, STATUS.CONNECTED)
                            and not map.is_blocked(x, y, h)
                            and current_intersection.streets[h] != STATUS.DEADEND]
            if valid_headings:
                # Calculate distances for valid headings
                heading_distances = []
                for heading in valid_headings:
                    potential_pos = get_potential_position(heading)
                    distance = math.sqrt((potential_pos[0] - goal[0])**2 + (potential_pos[1] - goal[1])**2)
                    heading_distances.append((heading, distance))
                best_heading, best_distance = min(heading_distances, key=lambda x: x[1])
                print(f"Aligning to valid street at heading {best_heading} (closest to goal, distance: {best_distance})")
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

    # Advanced Dijkstra logic: find the best intersection to explore next
    print("\nSearching for best intersection to explore using Dijkstra + Euclidean heuristic...")
    candidates = []
    for (tx, ty), inter in map.intersections.items():
        if any(inter.streets[h] in (STATUS.UNKNOWN, STATUS.UNEXPLORED) and not map.is_blocked(tx, ty, h) for h in range(8)):
            map.dijkstra(tx, ty)
            # Assume map.get_cost(tx, ty) returns the Dijkstra cost from current position to (tx, ty)
            cost_to_reach = getattr(map, 'get_cost', lambda tx, ty: abs(tx - x) + abs(ty - y))(tx, ty)
            heuristic_cost = math.sqrt((tx - goal[0]) ** 2 + (ty - goal[1]) ** 2)
            total_cost = cost_to_reach + heuristic_cost
            candidates.append(((tx, ty), total_cost))
    
    if candidates:
        best_goal, _ = min(candidates, key=lambda x: x[1])
        print(f"\nSetting goal to best intersection {best_goal} (lowest total cost)")
        map.dijkstra(*best_goal)
        if map.goal is not None:
            map.showwithrobot()
            step_toward_goal(map, behaviors)
            x, y, current_heading = map.pose()
            print(f"Moving to goal - Position: ({x}, {y}), Heading: {current_heading}")
            
            # Check if we've reached the goal intersection
            if (x, y) == goal:
                print("Goal intersection reached! Stopping exploration.")
                return
    else:
        # Check if we've explored all possible paths to the goal
        map.dijkstra(goal[0], goal[1])
        if map.goal is None:
            print("\nNo possible paths to goal intersection found. All possible paths have been exhausted.")
            print("Please enter a new command.")
            behaviors.drive.stop()  # Stop the robot
            return
        else:
            print("\nAll possible paths to goal have been explored. Please enter a new command.")
            behaviors.drive.stop()  # Stop the robot
            return



