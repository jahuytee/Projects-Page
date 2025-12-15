import math
from MapBuilding import STATUS
import time
from navigation import handle_deadend


# Helper to check if two intersections are adjacent (8-connected)
def fetch(NFCsensor, shared, map, behaviors, treasure=None):
    print("\n=== Starting Treasure Hunt ===")
    print(f"Initial robot state - Position: {map.pose()}")
    
    # Initialize the current intersection in the map
    x, y, h = map.pose()
    current_intersection = map.getintersection(x, y)
    print(f"Initializing intersection at ({x}, {y})")
    
    # Get treasure input from user if not provided
    if treasure is None:
        while True:
            try:
                treasure = int(input("Enter the treasure number to find (1-4): "))
                if 1 <= treasure <= 5:
                    break
                else:
                    print("Please enter a number between 1 and 4")
            except ValueError:
                print("Please enter a valid number")
    
    print(f"\nSearching for Treasure {treasure}")
    print("Initializing navigation...")

    if map.getintersection(x, y).streets[h] == STATUS.NONEXISTENT:
        turn_amt, actual_angle = behaviors.turning_behavior("left")
        map.markturn(turn_amt, actual_angle)
        
    result = behaviors.follow_line()
    x,y,h = map.pose()
    
    # Update position after initial follow_line
    if result == "intersection":
        map.update_connection()
        x,y,h = map.pose()
        print(f"After position update - Position: ({x}, {y}), Heading: {h}")
    
    has_street = False
    
    # Read NFC tag after follow_line in all cases
    current_id = NFCsensor.read()
    print(f"Current NFC ID: {current_id} at position ({x}, {y}) with heading {h}")
    
    if result == "intersection":
        print("Intersection detected")
        has_street = behaviors.pull_forward()
        print(f"Pull forward result: {has_street}")
        if has_street:
            inter = map.getintersection(x, y)
            map.setstreet(x, y, h, STATUS.UNEXPLORED)
            print(f"Marked street at heading {h} as UNEXPLORED")
            for delta in [-1, 1, 3, -3]:
                side_heading = (h + delta) % 8
                if inter.streets[side_heading] == STATUS.UNKNOWN:
                    inter.streets[side_heading] = STATUS.NONEXISTENT
                    print(f"Marked diagonal street at heading {side_heading} as NONEXISTENT")
        else:
            inter = map.getintersection(x, y)
            map.setstreet(x, y, h, STATUS.NONEXISTENT)
            print(f"Marked street at heading {h} as NONEXISTENT")
            for delta in [-3, 3]:
                side_heading = (h + delta) % 8
                if inter.streets[side_heading] == STATUS.UNKNOWN:
                    inter.streets[side_heading] = STATUS.NONEXISTENT
                    print(f"Marked diagonal street at heading {side_heading} as NONEXISTENT")
            
    elif result == "end":
        print("Dead end detected - executing U-turn")
        original_x, original_y, original_heading = map.markdeadend()
        time.sleep(0.1)
        handle_deadend(map, behaviors, original_x, original_y, original_heading)
        x, y, current_heading = map.pose()
        print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
        map.showwithrobot()  # Add map display after handling dead end

    current_x, current_y, current_heading = map.pose()
    current_intersection = map.intersections.get((current_x, current_y))
    print(f"\nCurrent intersection state:")
    print(f"Position: ({current_x}, {current_y})")
    print(f"Heading: {current_heading}")
    print(f"Intersection streets: {current_intersection.streets if current_intersection else 'None'}")

    # Get id and inter_prize_distance_dict from shared data
    inter_prize_distance_dict = shared.inter_prize_distance_dict

    current_dist = inter_prize_distance_dict[current_id][treasure]["distance"]
    print(f"Current distance to treasure: {current_dist}")

    # Track previous intersection's position and distance
    prev_x, prev_y = None, None
    prev_dist = None

    while True:
        print("\n=== Navigation Loop ===")
        if current_dist is None:
            print(f"Treasure {treasure} not found in the map.")
            return

        # Check if we've found two adjacent intersections with distance 0.5
        if current_dist == 0.5 and prev_dist == 0.5:
            # Verify that the intersections are adjacent
            if abs(current_x - prev_x) <= 1 and abs(current_y - prev_y) <= 1:
                print(f"Found treasure {treasure} between intersections ({prev_x}, {prev_y}) and ({current_x}, {current_y})")
                return
        
        # Turn through all headings to check for blockages
        for _ in range(4):
            if behaviors.check_blockage(h):
                print(f"Street at heading {h} is blocked")
                map.set_blocked(current_x, current_y, h, True)
                print(f"Marked blocked street at heading {h} as NONEXISTENT")
                map.showwithrobot()  # Add map display after marking blocked street
            turn_amt, actual_angle = behaviors.turning_behavior("left")
            map.markturn(turn_amt, actual_angle)
            h = (h + turn_amt) % 8
            print(f"Turned {turn_amt} units, new heading: {h}")
        
        possible_headings = [h for h in range(8)
                           if current_intersection.streets[h] not in (STATUS.DEADEND, STATUS.NONEXISTENT)
                           and not map.is_blocked(current_x, current_y, h)]
        
        print(f"Possible headings to explore: {possible_headings}")
        
        if not possible_headings:
            print("No more unblocked streets to explore")
            return
        
        # Find the best heading to explore
        best_heading = min(possible_headings,
                          key=lambda h: min((h - current_heading) % 8,
                                          (current_heading - h) % 8))
        print(f"Selected best heading: {best_heading}")
        
        # Turn to the best heading
        diff = (best_heading - current_heading) % 8
        counter_diff = (current_heading - best_heading) % 8
        turn_direction = "left" if diff <= counter_diff else "right"
        print(f"Turning {turn_direction} to reach heading {best_heading}")
        
        # Make the turn
        while current_heading != best_heading:
            turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
            map.markturn(turn_amt, actual_angle)
            current_heading = (current_heading + turn_amt) % 8
            print(f"Turned {turn_amt} units, new heading: {current_heading}")
        
        # Check for blockage after turning
        if behaviors.check_blockage(current_heading):
            print(f"Street at heading {current_heading} is blocked")
            map.set_blocked(current_x, current_y, current_heading, True)
            print(f"Marked blocked street at heading {current_heading} as NONEXISTENT")
            map.showwithrobot()  # Add map display after marking blocked street
            
            # Get new possible headings excluding the blocked one
            possible_headings = [h for h in range(8)
                               if current_intersection.streets[h] not in (STATUS.CONNECTED, STATUS.DEADEND, STATUS.NONEXISTENT)
                               and not map.is_blocked(current_x, current_y, h)]
            
            if not possible_headings:
                print("No more unblocked streets to explore")
                return
                
            # Find new best heading
            best_heading = min(possible_headings,
                              key=lambda h: min((h - current_heading) % 8,
                                              (current_heading - h) % 8))
            print(f"Selected new best heading: {best_heading}")
            
            # Turn to the new best heading
            diff = (best_heading - current_heading) % 8
            counter_diff = (current_heading - best_heading) % 8
            turn_direction = "left" if diff <= counter_diff else "right"
            print(f"Turning {turn_direction} to reach new heading {best_heading}")
            
            while current_heading != best_heading:
                turn_amt, actual_angle = behaviors.turning_behavior(turn_direction)
                map.markturn(turn_amt, actual_angle)
                current_heading = (current_heading + turn_amt) % 8
                print(f"Turned {turn_amt} units, new heading: {current_heading}")
        
        # Move forward to next intersection
        print("Following line to next intersection...")
        result = behaviors.follow_line()
        x,y,h = map.pose()
        print(f"After follow_line - Position: ({x}, {y}), Heading: {h}")
        
        # Add small delay to ensure sensor has time to stabilize
        time.sleep(0.1)
        
        # Read NFC tag after follow_line in all cases
        print("Reading NFC tag...")
        next_id = get_valid_nfc_id(NFCsensor, last_id=current_id)
        print(f"Next NFC ID: {next_id}")
        print(f"new distance is {inter_prize_distance_dict[next_id][treasure]['distance']}")
        
        map.update_connection()
        
        if result == "intersection":
            print("Intersection detected")
            has_street = behaviors.pull_forward()
            print(f"Pull forward result: {has_street}")
            if has_street:
                inter = map.getintersection(x, y)
                map.setstreet(x, y, h, STATUS.UNEXPLORED)
                print(f"Marked street at heading {h} as UNEXPLORED")
                for delta in [-1, 1, 3, -3]:
                    side_heading = (h + delta) % 8
                    if inter.streets[side_heading] == STATUS.UNKNOWN:
                        inter.streets[side_heading] = STATUS.NONEXISTENT
                        print(f"Marked diagonal street at heading {side_heading} as NONEXISTENT")
            else:
                inter = map.getintersection(x, y)
                map.setstreet(x, y, h, STATUS.NONEXISTENT)
                print(f"Marked street at heading {h} as NONEXISTENT")
                for delta in [-3, 3]:
                    side_heading = (h + delta) % 8
                    if inter.streets[side_heading] == STATUS.UNKNOWN:
                        inter.streets[side_heading] = STATUS.NONEXISTENT
                        print(f"Marked diagonal street at heading {side_heading} as NONEXISTENT")
        
            map.showwithrobot()  # Add map display after updating connection
            # Update current intersection state and continue to next iteration
            current_intersection = inter
            continue
        elif result == "end":
            print("Dead end detected - executing U-turn")
            original_x, original_y, original_heading = map.markdeadend()
            time.sleep(0.1)
            handle_deadend(map, behaviors, original_x, original_y, original_heading)
            x, y, current_heading = map.pose()
            print(f"After U-turn - Position: ({x}, {y}), Heading: {current_heading}")
            map.showwithrobot()  # Add map display after handling dead end

        x, y, current_heading = map.pose()
        next_intersection = map.intersections.get((x, y))
        print(f"\nNext intersection state:")
        print(f"Position: ({x}, {y})")
        print(f"Heading: {current_heading}")
        print(f"Intersection streets: {next_intersection.streets if next_intersection else 'None'}")
        
        next_dist = inter_prize_distance_dict[next_id][treasure]["distance"]
        print(f"Next distance to treasure: {next_dist}")
        
        # Check if the next distance is None (treasure unreachable from here)
        if next_dist is None:
            print(f"Treasure {treasure} not reachable from intersection ({x}, {y}).")
            return
            
        # If we've found a better distance or equal distance, continue exploring
        if next_dist <= current_dist:
            # Update previous intersection info before moving to next
            prev_x, prev_y = current_x, current_y
            prev_dist = current_dist
            
            current_x, current_y = x, y
            current_intersection = next_intersection
            current_dist = next_dist
            print(f"Moving to intersection ({x}, {y}) with distance {next_dist}")
        else:
            # If distance is greater, backtrack to previous intersection
            print(f"Distance increased from {current_dist} to {next_dist}, backtracking")
            
            # Set the previous intersection as the goal
            print(f"Setting goal to previous intersection ({prev_x}, {prev_y})")
            map.dijkstra(prev_x, prev_y)
            
            if map.goal is None:
                print("No valid path found to previous intersection")
                return
                
            # Navigate back to previous intersection
            print("Navigating back to previous intersection...")
            while map.goal is not None:
                map.showwithrobot()
                map.step_toward_goal(map, behaviors)
                x, y, current_heading = map.pose()
                print(f"Moving to goal - Position: ({x}, {y}), Heading: {current_heading}")
                
                # Check if we've reached the previous intersection
                if (x, y) == (prev_x, prev_y):
                    print("Reached previous intersection")
                    break
            
            # Update current state to match previous intersection
            current_x, current_y = prev_x, prev_y
            current_intersection = map.getintersection(current_x, current_y)
            current_dist = prev_dist
            print(f"Back at previous intersection - Position: ({current_x}, {current_y}), Distance: {current_dist}")






import time

def get_valid_nfc_id(NFCsensor, last_id=None, timeout=1.0):
    """Wait for a new NFC tag different from last_id, or timeout."""
    start = time.time()
    while time.time() - start < timeout:
        tag = NFCsensor.read()
        if tag is not None and tag != last_id:
            return tag
        time.sleep(0.05)
    print("Warning: No new NFC tag detected within timeout.")
    return last_id  # fallback to last_id if nothing new
    
    




    









    
