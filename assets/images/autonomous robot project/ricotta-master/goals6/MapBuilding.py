import os
import time
import pickle
from enum import Enum
import matplotlib.pyplot as plt
import traceback


class STATUS(Enum):
    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    DEADEND = 3
    CONNECTED = 4


class Intersection:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.streets = [STATUS.UNKNOWN for i in range(8)]
        self.cost = float('inf')
        self.direction = None

        
# load map from file if it exists, otherwise create a new map
def prompt_and_load_map():
    filename = input("Enter filename to load (e.g. mymap.pickle): ").strip()
    try:
        with open(filename, 'rb') as file:
            map = pickle.load(file)
        print(f"Map loaded from {filename}.")
        x = int(input("Enter current x position of robot: "))
        y = int(input("Enter current y position of robot: "))
        h = int(input("Enter current heading (0-7): "))
        map.x, map.y, map.heading = x, y, h
        return map
    except FileNotFoundError:
        print(f"File {filename} not found. Starting with a blank map.")
    except Exception as e:
        print(f"Failed to load map: {e}")
        print("Starting with a blank map instead.")

class Map:
    # Define the heading to delta mapping
    heading_to_delta = {
        0: (0, 1), 
        1: (-1, 1), 
        2: (-1, 0), 
        3: (-1, -1),
        4: (0, -1), 
        5: (1, -1), 
        6: (1, 0), 
        7: (1, 1)
    }

    # Define the heading vectors for plotting
    heading_vectors = {
        0: (0.0, 0.5), 
        1: (-0.5, 0.5), 
        2: (-0.5, 0.0), 
        3: (-0.5, -0.5),
        4: (0.0, -0.5), 
        5: (0.5, -0.5), 
        6: (0.5, 0.0), 
        7: (0.5, 0.5)
    }

    # defines the color mapping for each status
    color_map = {
        STATUS.UNKNOWN: 'black',
        STATUS.NONEXISTENT: 'lightgray',
        STATUS.UNEXPLORED: 'blue',
        STATUS.DEADEND: 'red',
        STATUS.CONNECTED: 'green'
    }
    def __init__(self):
        self.x = 0
        self.y = 0
        self.heading = 0
        self.intersections = {}
        
        self.cost = None
        self.direction = None
        self.goal = None

    def pose(self):
        return (self.x, self.y, self.heading)

    def calcmove(self):
        dx, dy = self.heading_to_delta[self.heading]
        self.x += dx
        self.y += dy

    def calcturn(self, turn_amount):
        # Store previous heading where we detected a line
        prev_heading = self.heading
        
        # Calculate intended and new headings
        intended_heading = (self.heading + turn_amount) % 8
        new_heading = (self.heading + turn_amount) % 8
        
        # Check if we're at a known intersection and the new heading points to a nonexistent street
        current = self.getintersection(self.x, self.y)
        if current and current.streets[new_heading] == STATUS.NONEXISTENT:
            print(f"Warning: Heading {new_heading} points to nonexistent street")
            
            # Find valid headings (not NONEXISTENT)
            valid_headings = [h for h in range(8) if current.streets[h] != STATUS.NONEXISTENT]
            
            if valid_headings:
                # First check if our previous heading is valid (we know there's a street there)
                if prev_heading in valid_headings:
                    # Only use previous heading if we're within ±2 steps of it
                    diff_from_prev = min((new_heading - prev_heading) % 8, 
                                       (prev_heading - new_heading) % 8)
                    if diff_from_prev <= 2:
                        print(f"Correcting to previous valid heading: {prev_heading}")
                        self.heading = prev_heading
                        return
                
                # Find headings within ±1 step of where we detected a line
                close_connected = []  # CONNECTED streets within ±1
                close_unexplored = [] # UNEXPLORED streets within ±1
                close_deadend = []    # DEADEND streets within ±1
                for h in valid_headings:
                    diff = min((h - new_heading) % 8, (new_heading - h) % 8)
                    if diff <= 1:
                        if current.streets[h] == STATUS.CONNECTED:
                            close_connected.append(h)
                        elif current.streets[h] == STATUS.UNEXPLORED:
                            close_unexplored.append(h)
                        elif current.streets[h] == STATUS.DEADEND:
                            close_deadend.append(h)
                
                # If we have any CONNECTED streets within ±1, prefer those
                if close_connected:
                    # Choose the one we would have encountered first in our turn direction
                    if turn_amount < 0:
                        chosen_heading = max(close_connected)
                    else:
                        chosen_heading = min(close_connected)
                    print(f"Correcting to nearest CONNECTED heading: {chosen_heading}")
                    self.heading = chosen_heading
                    return
                # Otherwise use UNEXPLORED streets within ±1
                elif close_unexplored:
                    if turn_amount < 0:
                        chosen_heading = max(close_unexplored)
                    else:
                        chosen_heading = min(close_unexplored)
                    print(f"Correcting to nearest UNEXPLORED heading: {chosen_heading}")
                    self.heading = chosen_heading
                    return
                # Then consider DEADEND streets within ±1
                elif close_deadend:
                    if turn_amount < 0:
                        chosen_heading = max(close_deadend)
                    else:
                        chosen_heading = min(close_deadend)
                    print(f"Correcting to nearest DEADEND heading: {chosen_heading}")
                    self.heading = chosen_heading
                    return
                    
                # If no close headings, find valid headings in the turn direction
                valid_connected = []  # CONNECTED streets in turn direction
                valid_unexplored = [] # UNEXPLORED streets in turn direction
                valid_deadend = []    # DEADEND streets in turn direction
                for h in valid_headings:
                    # Calculate turn needed to reach this heading
                    turn_needed = (h - prev_heading) % 8
                    if turn_needed > 4:
                        turn_needed -= 8
                    # Check if turn direction matches intended direction
                    if (turn_needed > 0 and turn_amount > 0) or (turn_needed < 0 and turn_amount < 0):
                        if current.streets[h] == STATUS.CONNECTED:
                            valid_connected.append(h)
                        elif current.streets[h] == STATUS.UNEXPLORED:
                            valid_unexplored.append(h)
                        elif current.streets[h] == STATUS.DEADEND:
                            valid_deadend.append(h)
                
                # Prefer CONNECTED streets in turn direction
                if valid_connected:
                    if turn_amount < 0:
                        chosen_heading = max(valid_connected)
                    else:
                        chosen_heading = min(valid_connected)
                    print(f"Correcting to first CONNECTED heading in turn direction: {chosen_heading}")
                    self.heading = chosen_heading
                    return
                # Fall back to UNEXPLORED streets in turn direction
                elif valid_unexplored:
                    if turn_amount < 0:
                        chosen_heading = max(valid_unexplored)
                    else:
                        chosen_heading = min(valid_unexplored)
                    print(f"Correcting to first UNEXPLORED heading in turn direction: {chosen_heading}")
                    self.heading = chosen_heading
                    return
                # Consider DEADEND streets in turn direction
                elif valid_deadend:
                    if turn_amount < 0:
                        chosen_heading = max(valid_deadend)
                    else:
                        chosen_heading = min(valid_deadend)
                    print(f"Correcting to first DEADEND heading in turn direction: {chosen_heading}")
                    self.heading = chosen_heading
                    return
                
                # If no valid headings in turn direction, just take closest
                closest = min(valid_headings,
                            key=lambda h: min((h - new_heading) % 8,
                                            (new_heading - h) % 8))
                print(f"Adjusting to nearest valid heading: {closest}")
                self.heading = closest
                return
                
        # If no adjustment needed or possible, use the original new heading
        self.heading = new_heading

    def calcuturn(self):
        self.heading = (self.heading + 4) % 8
        

    def getintersection(self, x, y):
        if (x, y) not in self.intersections:
            self.intersections[(x, y)] = Intersection(x, y)
        return self.intersections[(x, y)]
    
    def has_intersection(self, x, y):
        return (x, y) in self.intersections

    def setstreet(self, x, y, heading, status):
        # Set status for current intersection
        inter = self.getintersection(x, y)
        inter.streets[heading] = status

        # Also update the reverse direction of the neighbor
        dx, dy = self.heading_to_delta[heading]
        nx, ny = x + dx, y + dy
        if (nx, ny) in self.intersections:
            neighbor = self.intersections[(nx, ny)]
            neighbor.streets[(heading + 4) % 8] = status


    def markturn(self, turn_amount, actual_angle=None):
        current = self.getintersection(self.x, self.y)
        prev_heading = (self.heading - turn_amount) % 8
        if current.streets[self.heading] == STATUS.UNKNOWN:
            current.streets[self.heading] = STATUS.UNEXPLORED

        skipped = []
        if turn_amount > 0:
            for i in range(1, turn_amount):
                skipped.append((prev_heading + i) % 8)
        elif turn_amount < 0:
            for i in range(-1, turn_amount, -1):
                skipped.append((prev_heading + i) % 8)

        for d in skipped:
            if current.streets[d] in (STATUS.UNKNOWN, STATUS.UNEXPLORED):
                current.streets[d] = STATUS.NONEXISTENT

        # Check if we landed on a NONEXISTENT street
        if current.streets[self.heading] == STATUS.NONEXISTENT and actual_angle is not None:
            # Check neighboring headings
            left_heading = (self.heading - 1) % 8
            right_heading = (self.heading + 1) % 8
            
            # Only proceed if both neighbors are valid streets
            if (current.streets[left_heading] in (STATUS.CONNECTED, STATUS.UNEXPLORED) and 
                current.streets[right_heading] in (STATUS.CONNECTED, STATUS.UNEXPLORED)):
                
                # Calculate angles to both valid headings (convert from 45° steps to actual degrees)
                left_angle = (prev_heading * 45 + (left_heading - prev_heading) % 8 * 45) % 360
                right_angle = (prev_heading * 45 + (right_heading - prev_heading) % 8 * 45) % 360
                
                # Calculate absolute differences (considering wrap-around)
                left_diff = min((actual_angle - left_angle) % 360, (left_angle - actual_angle) % 360)
                right_diff = min((actual_angle - right_angle) % 360, (right_angle - actual_angle) % 360)
                
                # Choose the closer heading
                if left_diff < right_diff:
                    print(f"Correcting heading from {self.heading} to {left_heading} based on weighted angle sensor (actual={actual_angle:.1f}°, target={left_angle}°)")
                    self.heading = left_heading
                else:
                    print(f"Correcting heading from {self.heading} to {right_heading} based on weighted angle sensor (actual={actual_angle:.1f}°, target={right_angle}°)")
                    self.heading = right_heading

    def update_connection(self):
        dx, dy = self.heading_to_delta[self.heading]
        next_x = self.x + dx
        next_y = self.y + dy
        heading = self.heading

        # Get current intersection and check if the street is already marked as DEADEND
        current = self.getintersection(self.x, self.y)
        next_inter = self.getintersection(next_x, next_y)
        
        # Only mark as CONNECTED if not already DEADEND
        if current.streets[heading] != STATUS.DEADEND:
            current.streets[heading] = STATUS.CONNECTED
            
        # Same for the reverse direction from the next intersection
        reverse_heading = (heading + 4) % 8
        if next_inter.streets[reverse_heading] != STATUS.DEADEND:
            next_inter.streets[reverse_heading] = STATUS.CONNECTED

        # Then update the robot's internal position
        self.x = next_x
        self.y = next_y

    def markdeadend(self, behaviors):
        print("Reached end of the street.")
        current_intersection = self.getintersection(self.x, self.y)
        # Mark current heading as deadend
        current_intersection.streets[self.heading] = STATUS.DEADEND


        print("Performing U-turn")
        # Make the robot physically turn 180 degrees
        behaviors.turning_behavior("left")
        # Update internal heading
        self.calcuturn()

        print("Following line back to intersection")
        result_back = behaviors.follow_line()

        if result_back == "intersection":
            print("Intersection reached again after U-turn")
            has_street = behaviors.pull_forward()
            # Stop the robot after pulling forward
            behaviors.drive.stop()

            old_x, old_y, old_heading = self.x, self.y, self.heading
            prev_intersection = self.getintersection(old_x, old_y)
            new_intersection = self.getintersection(self.x, self.y)
        else:
            print("Warning: expected to reach intersection but didn't.")
        return has_street

    def showwithrobot(self):
        self.show()  
        dx, dy = self.heading_vectors[self.heading]
        plt.arrow(self.x, self.y, dx, dy,
                width=0.2, head_width=0.3, head_length=0.1, color='magenta')

        plt.pause(0.001)

    def show(self):
        plt.clf()
        plt.axes()
        plt.gca().set_xlim(-3.5, 3.5)
        plt.gca().set_ylim(-3.5, 3.5)
        plt.gca().set_aspect('equal')

        for x in range(-3, 4):
            for y in range(-3, 4):
                plt.plot(x, y, color='lightgray', marker='o', markersize=8)

        for (ix, iy), intersection in self.intersections.items():
            for idx in range(8):
                dx, dy = self.heading_vectors[idx]
                status = intersection.streets[idx]
                color = self.color_map[status]
                plt.plot([ix, ix + dx], [iy, iy + dy], color=color)
        
        # draw optimal path to goal if it exists
        if self.goal is not None and (self.x, self.y) in self.intersections:
            cx, cy = self.x, self.y
            current = self.intersections[(cx, cy)]
            visited = set()

            while current.direction is not None and (cx, cy) != self.goal:
                visited.add((cx, cy))
                heading = current.direction
                dx, dy = self.heading_vectors[heading]
                nx, ny = cx + dx, cy + dy

                # Draw the segment in a distinct color 
                plt.plot([cx, nx], [cy, ny], color='orange', linewidth=3.5)

                # Move to next
                cx, cy = int(round(nx)), int(round(ny))
                if (cx, cy) in visited or (cx, cy) not in self.intersections:
                    break  # stop if we're looping or hit a node that doesn't exist
                current = self.intersections[(cx, cy)]
                

        plt.pause(0.001)

    def dijkstra(self, xgoal, ygoal):
        # Reset all previous cost/direction info
        for inter in self.intersections.values():
            inter.cost = float('inf')  # infinity
            inter.direction = None

        self.goal = (xgoal, ygoal)  

        # initialize goal node
        goal = self.getintersection(xgoal, ygoal)
        goal.cost = 0
        goal.direction = None

        # initialize the onDeck queue with just the goal
        onDeck = [goal]

        # Helper func to insert in sorted order
        def sortedInsert(queue, inter):
            for i in range(len(queue)):
                if queue[i].cost > inter.cost:
                    queue.insert(i, inter)
                    return
            queue.append(inter)

        while onDeck:
            current = onDeck.pop(0)  # pop lowest-cost leaf
            x, y = current.x, current.y

            for heading in range(8):
                if current.streets[heading] != STATUS.CONNECTED:
                    continue  #

                dx, dy = self.heading_to_delta[heading]
                nx, ny = x + dx, y + dy
                if (nx, ny) not in self.intersections:
                    continue  # Do not create new intersection while exploring
                neighbor = self.intersections[(nx, ny)]

                # we change the cost based off the heading, which tells us 
                # if we are moving straight or diagonally
                if heading % 2 == 0:  
                    step_cost = 1
                else:  
                    step_cost = 2**0.5

                potential_cost = current.cost + step_cost

                # found a better path to neighbor
                if potential_cost < neighbor.cost:
                    # remove neighbor from onDeck if it's already in there
                    if neighbor.cost < float('inf'):
                        try:
                            onDeck.remove(neighbor)
                        except ValueError:
                            pass  

                    # save cost/direction
                    neighbor.cost = potential_cost
                    neighbor.direction = (heading + 4) % 8  # point back toward current

                    # re-insert into onDeck in sorted order
                    sortedInsert(onDeck, neighbor)

    # clear the goal and reset all intersections to unknown
    # this is called when the goal is reached or when the user wants to clear the goal
    def cleargoal(self):
        self.goal = None
        for inter in self.intersections.values():
            inter.cost = float('inf')
            inter.direction = None


    # update the Brain to Include Automatic Driving to a Goal
    # for calling goto function in brain
    def step_toward_goal(self, behaviors):
        x, y, h = self.pose()
        inter = self.getintersection(x, y)

        # if the robot is at the goal, clear the goal and return
        if (x, y) == self.goal:
            print("Goal reached!")
            self.cleargoal()
            return

        # if there are no known paths to goal
        direction = inter.direction
        if direction is None:
            print("No known path to goal.")
            return

        # if the robot is facing the goal, move forward
        if direction == h:
            print("Advancing toward goal...")
            result = behaviors.follow_line()
            # if the robot is at an intersection, pull forward
            if result == "intersection":
                has_street = behaviors.pull_forward()
                self.update_connection()
            return

                    
        # Decide turn direction
        turn_amt = (direction - h) % 8
        if turn_amt > 4:
            turn_amt -= 8
        turn_dir = "left" if turn_amt > 0 else "right"

        # Start turning, one step (45°) at a time, until at correct heading
        while self.heading != direction:
            print(f"Current heading: {self.heading}, desired: {direction}")

            # Turn one 45° step
            step = 1 if turn_dir == "left" else -1
            result = behaviors.turning_behavior(turn_dir)  # should return number of steps taken (±1)
            self.calcturn(result)
            self.markturn(result)

            # Read sensors after turn
            L, M, R = behaviors.sensor.read()

            # Check for a valid street at new heading using sensor AND map
            inter = self.getintersection(self.x, self.y)
            is_on_line = (L, M, R) != (0, 0, 0)
            valid_street = inter.streets[self.heading] == STATUS.CONNECTED

            print(f"Sensor sees line: {is_on_line}, map says street at heading {self.heading} is {inter.streets[self.heading].name}")

            if self.heading == direction and is_on_line and valid_street:
                print("Aligned to correct heading with valid street — done turning.")
                break
            elif is_on_line:
                print("At wrong street. Pausing briefly before continuing turn...")
                behaviors.drive.stop()
                time.sleep(0.3)  # Let robot sit on wrong street before resuming

        # Once aligned, drive forward
        print("Heading alignment complete. Driving forward.")
        result = behaviors.follow_line()
        if result == "intersection":
            has_street = behaviors.pull_forward()
            self.update_connection()

    def get_next_action(self):
        """
        Returns the next action the brain should take based on current map state.
        Returns a tuple of (action_type, action_params)
        """
        x, y, h = self.pose()
        current_intersection = self.getintersection(x, y)

        # If we have a goal, get next step toward it
        if self.goal is not None:
            if (x, y) == self.goal:
                self.cleargoal()
                return ("goal_reached", None)

            direction = current_intersection.direction
            if direction is None:
                return ("no_path_to_goal", None)

            if direction == h:
                return ("go_straight", None)
            else:
                turn_amt = (direction - h) % 8
                if turn_amt > 4:
                    turn_amt -= 8
                turn_dir = "left" if turn_amt > 0 else "right"
                return ("turn", {"direction": turn_dir, "steps": 1})

        # Check for UNKNOWN streets
        unknown_headings = []
        for heading in range(8):
            if current_intersection.streets[heading] == STATUS.UNKNOWN:
                unknown_headings.append(heading)

        if unknown_headings:
            best_heading = min(unknown_headings, 
                            key=lambda h: min(
                                (h - h) % 8,
                                (h - h) % 8
                            ))
            diff = (best_heading - h) % 8
            if diff <= 4:
                turn_direction = "right"
                turns_needed = diff
            else:
                turn_direction = "left"
                turns_needed = 8 - diff
            return ("explore_unknown", {"heading": best_heading, "turn_direction": turn_direction, "turns_needed": turns_needed})

        # ... similar logic for UNEXPLORED streets ...

    def handle_action_result(self, action_type, result):
        """
        Updates map state based on the result of an action taken by the brain.
        """
        x, y, h = self.pose()
        current_intersection = self.getintersection(x, y)

        if action_type == "go_straight":
            if result == "intersection":
                self.update_connection()
                return True
            elif result == "end":
                current_intersection.streets[h] = STATUS.DEADEND
                for direction, status in enumerate(current_intersection.streets):
                    if status == STATUS.UNEXPLORED:
                        current_intersection.streets[direction] = STATUS.NONEXISTENT
                return True
            return False

        # ... handle other action types ...




