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
        self.blocked = [False for _ in range(8)]  # Blocked flags for each direction
        self.cost = float('inf')
        self.direction = None

    def set_blocked(self, heading, value: bool):
        self.blocked[heading] = value

    def is_blocked(self, heading) -> bool:
        return self.blocked[heading]

        
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
        # Never overwrite a DEADEND status
        if inter.streets[heading] != STATUS.DEADEND:
            inter.streets[heading] = status

        # Also update the reverse direction of the neighbor
        dx, dy = self.heading_to_delta[heading]
        nx, ny = x + dx, y + dy
        if (nx, ny) in self.intersections:
            neighbor = self.intersections[(nx, ny)]
            # Never overwrite a DEADEND status
            if neighbor.streets[(heading + 4) % 8] != STATUS.DEADEND:
                neighbor.streets[(heading + 4) % 8] = status


    def markturn(self, turn_amount, actual_angle=None):
        current = self.getintersection(self.x, self.y)
        
        prev_heading = self.heading
        # prev_heading = (self.heading - turn_amount) % 8
        new_heading = (self.heading + turn_amount) % 8


        # Check if we landed on a NONEXISTENT street
        if current.streets[new_heading] == STATUS.NONEXISTENT and actual_angle is not None:
            # Only consider headings ±1 away from current heading
            valid_headings = []
            for delta in [-2,-1,1,2]:
                h = (new_heading + delta) % 8
                if current.streets[h] != STATUS.NONEXISTENT:
                    valid_headings.append(h)
            print(f"Valid headings: {valid_headings}")
            
            if valid_headings:
                # Find the heading closest to the measured angle
                print(f"Valid headings: {valid_headings}")
                best_heading = None
                min_diff = 360
                
                for h in valid_headings:
                    # Calculate degrees for this heading (0=0°, 1=45°, 2=90°, etc)
                    # degrees = h * 45
                    heading = h
                    if heading == 0:
                        heading = 8
                    
                    degrees = abs((prev_heading - heading) * 45) % 360
                    # If it's a right turn (negative angle), make the degrees negative
                    if actual_angle < 0:
                        degrees = -degrees
                    
                    # Calculate angle difference
                    diff = abs(degrees - actual_angle)
                    print(f"Comparing heading {h} ({degrees}°) with measured {actual_angle}° - diff = {diff}°, status={current.streets[h]}")
                    
                    if diff < min_diff:
                        min_diff = diff
                        best_heading = h
        
                    
                print(f"Correcting heading from {self.heading} to {best_heading} based on angle sensor (measured={actual_angle:.1f}°, chosen={degrees}°)")
                # If we're turning onto an UNKNOWN street, mark it as UNEXPLORED
                if (current.streets[best_heading] == STATUS.NONEXISTENT) or (current.streets[best_heading] == STATUS.UNKNOWN): 
                    current.streets[best_heading] = STATUS.UNEXPLORED
                self.heading = best_heading
                    
                    
        else:
            # If no correction needed and we're on an UNKNOWN street, mark it as UNEXPLORED
            if current.streets[new_heading] == STATUS.UNKNOWN:
                current.streets[new_heading] = STATUS.UNEXPLORED
            self.heading = new_heading

            

        skipped = []
        if turn_amount > 0:
            # Start from one after prev_heading to avoid marking it
            for i in range(1, turn_amount):
                skipped_heading = (prev_heading + i) % 8
                if skipped_heading != prev_heading and skipped_heading != self.heading:  # Don't include the heading we started from or ended at
                    skipped.append(skipped_heading)
        elif turn_amount < 0:
            # Start from one after prev_heading to avoid marking it
            for i in range(-1, turn_amount, -1):
                skipped_heading = (prev_heading + i) % 8
                if skipped_heading != prev_heading and skipped_heading != self.heading:  # Don't include the heading we started from or ended at
                    skipped.append(skipped_heading)

        for d in skipped:
            if current.streets[d] in (STATUS.UNKNOWN, STATUS.UNEXPLORED):
                current.streets[d] = STATUS.NONEXISTENT

    def possible_angles(self, heading, turn_amount=None):
        inter = self.getintersection(self.x, self.y)
        possible_angles = []
        for i in range(len(inter.streets)):
            if inter.streets[i] != STATUS.NONEXISTENT:
                angle = (i - heading) * 45
                if angle == 0:
                    continue  # skip straight ahead
                possible_angles.append(angle)
        return possible_angles

    def update_connection(self):
        dx, dy = self.heading_to_delta[self.heading]
        next_x = self.x + dx
        next_y = self.y + dy
        heading = self.heading

        # Mark the forward direction as CONNECTED (from current to next)
        current = self.getintersection(self.x, self.y)
        if current.streets[heading] != STATUS.DEADEND:
            current.streets[heading] = STATUS.CONNECTED
            
        # Mark the reverse direction as CONNECTED (from next back to current)
        reverse_heading = (heading + 4) % 8
        next_inter = self.getintersection(next_x, next_y)
        if next_inter.streets[reverse_heading] != STATUS.DEADEND:
            next_inter.streets[reverse_heading] = STATUS.CONNECTED

        # Update robot's position
        self.x = next_x
        self.y = next_y

    def markdeadend(self, behaviors):
        print("Reached end of the street.")
        current_intersection = self.getintersection(self.x, self.y)
        # Only mark as DEADEND if it wasn't already marked as NONEXISTENT
        if current_intersection.streets[self.heading] != STATUS.NONEXISTENT:
            current_intersection.streets[self.heading] = STATUS.DEADEND

        # Store the original position and heading before U-turn
        original_x, original_y = self.x, self.y
        original_heading = self.heading
        print(f"[DEBUG] Before U-turn: x={original_x}, y={original_y}, heading={original_heading}")

        print("Performing U-turn")
        # Make the robot physically turn 180 degrees
        behaviors.turning_behavior("left")
        # Update internal heading

        print("Following line back to intersection")
        result_back = behaviors.follow_line()
        print(f"[DEBUG] After follow_line: x={self.x}, y={self.y}, heading={self.heading}")

        self.calcuturn()
        print(f"[DEBUG] After U-turn: x={self.x}, y={self.y}, heading={self.heading}")
        has_street = False  # Initialize has_street to False by default

        if result_back == "intersection":
            print("Intersection reached again after U-turn")
            has_street = behaviors.pull_forward()
            print(f"[DEBUG] After pull_forward: x={self.x}, y={self.y}, heading={self.heading}")
            # Stop the robot after pulling forward
            behaviors.drive.stop()

            # If we're back at the original intersection, don't mark any new deadends
            if (self.x, self.y) == (original_x, original_y):
                print("Back at original intersection - not marking new deadends")
            else:
                print(f"[WARNING] Robot did not return to original intersection! Now at x={self.x}, y={self.y}, heading={self.heading}")
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
                linewidth = 2
                if hasattr(intersection, 'blocked') and intersection.blocked[idx]:
                    color = 'purple'
                    linewidth = 4
                plt.plot([ix, ix + dx], [iy, iy + dy], color=color, linewidth=linewidth)
        
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
        # First check if the goal intersection exists
        if (xgoal, ygoal) not in self.intersections:
            print(f"Error: Goal intersection ({xgoal}, {ygoal}) does not exist in the map.")
            self.goal = None
            return

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
                # Only consider connected streets that are not blocked
                if current.streets[heading] != STATUS.CONNECTED:
                    continue
                if hasattr(current, 'blocked') and current.blocked[heading]:
                    continue  # Skip blocked streets

                dx, dy = self.heading_to_delta[heading]
                nx, ny = x + dx, y + dy
                if (nx, ny) not in self.intersections:
                    continue  # Skip if neighbor intersection doesn't exist
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
            # Check for blockage before moving
            blocked = behaviors.check_blockage()
            if blocked:
                print("Blocked street detected! Replanning path...")
                self.set_blocked(x, y, h, True)
                # Replan path to goal
                self.dijkstra(self.goal[0], self.goal[1])
                # Check if a valid path was found after replanning
                if inter.direction is None:
                    print("No possible route to replan to - goal is unreachable!")
                    self.cleargoal()
                    return
                return
                
            result = behaviors.follow_line()
            # if the robot is at an intersection, pull forward
            if result == "intersection":
                has_street = behaviors.pull_forward()
                if has_street:
                    for delta in [-1,1,3,-3]:
                        side_heading = (h + delta) % 8
                        if inter.streets[side_heading] == STATUS.UNKNOWN:
                            inter.streets[side_heading] = STATUS.NONEXISTENT
                else:
                    for delta in [-3,3]:
                        side_heading = (h + delta) % 8
                        if inter.streets[side_heading] == STATUS.UNKNOWN:
                            inter.streets[side_heading] = STATUS.NONEXISTENT
                self.update_connection()  # Move this after pull_forward and diagonal marking
            return

        # Decide turn direction
        turn_amt = (direction - h) % 8
        if turn_amt > 4:
            turn_amt -= 8
        turn_dir = "left" if turn_amt > 0 else "right"

        # Start turning, one step (45) at a time, until at correct heading
        while self.heading != direction:
            print(f"Current heading: {self.heading}, desired: {direction}")

            # Turn one 45 step
            step = 1 if turn_dir == "left" else -1
            turn_result = behaviors.turning_behavior(turn_dir)  # should return number of steps taken (±1)
            
            turn_amount, actual_angle = turn_result
            self.markturn(turn_amount, actual_angle)
    
            # Read sensors after turn
            L, M, R = behaviors.sensor.read()

            # Check for a valid street at new heading using sensor AND map
            inter = self.getintersection(self.x, self.y)
            is_on_line = (L, M, R) != (0, 0, 0)
            valid_street = inter.streets[self.heading] == STATUS.CONNECTED

            # Check for blockage after turn
            blocked = behaviors.check_blockage()
            if blocked:
                print("Blocked street detected! Replanning path...")
                self.set_blocked(self.x, self.y, self.heading, True)
                # Replan path to goal
                self.dijkstra(self.goal[0], self.goal[1])
                # Check if a valid path was found after replanning
                if inter.direction is None:
                    print("No possible route to replan to - goal is unreachable!")
                    self.cleargoal()
                    return
                return

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
            if has_street:
                for delta in [-1,1,3,-3]:
                    side_heading = (self.heading + delta) % 8
                    if inter.streets[side_heading] == STATUS.UNKNOWN:
                        inter.streets[side_heading] = STATUS.NONEXISTENT
            else:
                for delta in [-3,3]:
                    side_heading = (self.heading + delta) % 8
                    if inter.streets[side_heading] == STATUS.UNKNOWN:
                        inter.streets[side_heading] = STATUS.NONEXISTENT
            self.update_connection()  # Move this after pull_forward and diagonal marking

    def set_pose(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

    def set_position(self, x, y):
        self.x = x
        self.y = y

    def set_heading(self, heading):
        self.heading = heading

    def set_blocked(self, x, y, heading, value: bool):
        inter = self.getintersection(x, y)
        inter.set_blocked(heading, value)

    def is_blocked(self, x, y, heading) -> bool:
        inter = self.getintersection(x, y)
        return inter.is_blocked(heading)






