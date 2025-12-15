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
        # Never overwrite a DEADEND or CONNECTED status
        if inter.streets[heading] not in (STATUS.DEADEND, STATUS.CONNECTED):
            inter.streets[heading] = status

        # Also update the reverse direction of the neighbor
        dx, dy = self.heading_to_delta[heading]
        nx, ny = x + dx, y + dy
        if (nx, ny) in self.intersections:
            neighbor = self.intersections[(nx, ny)]
            # Never overwrite a DEADEND or CONNECTED status
            if neighbor.streets[(heading + 4) % 8] not in (STATUS.DEADEND, STATUS.CONNECTED):
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
            
            
            if valid_headings:
                # Find the heading closest to the measured angle
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

        for delta in [-1, 1]:
            diag = (self.heading + delta) % 8
            self.setstreet(self.x, self.y, diag, STATUS.NONEXISTENT)

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

    def markdeadend(self):
        print("Reached end of the street.")
        current_intersection = self.getintersection(self.x, self.y)
        # Only mark as DEADEND if it wasn't already marked as NONEXISTENT
        if current_intersection.streets[self.heading] != STATUS.NONEXISTENT:
            current_intersection.streets[self.heading] = STATUS.DEADEND

        # Store the original position and heading before U-turn
        original_x, original_y = self.x, self.y
        original_heading = self.heading
        print(f"[DEBUG] Before U-turn: x={original_x}, y={original_y}, heading={original_heading}")

        # Return the original position and heading
        return original_x, original_y, original_heading

    

    def showwithrobot(self):
        self.show()  
        dx, dy = self.heading_vectors[self.heading]
        plt.arrow(self.x, self.y, dx, dy,
                width=0.2, head_width=0.3, head_length=0.1, color='magenta')

        plt.pause(0.001)

    def show(self):
        plt.clf()
        plt.axes()
        plt.gca().set_xlim(-5, 5)
        plt.gca().set_ylim(-5, 5)
        plt.gca().set_aspect('equal')

        for x in range(-4, 5):
            for y in range(-4, 5):
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
        # Never mark a DEADEND or NONEXISTENT street as blocked
        if inter.streets[heading] not in (STATUS.DEADEND, STATUS.NONEXISTENT):
            inter.set_blocked(heading, value)
            # Also block the reverse direction at the neighbor intersection
            dx, dy = self.heading_to_delta[heading]
            nx, ny = x + dx, y + dy
            if (nx, ny) in self.intersections:
                neighbor = self.getintersection(nx, ny)
                reverse_heading = (heading + 4) % 8
                # Never mark a DEADEND or NONEXISTENT street as blocked
                if neighbor.streets[reverse_heading] not in (STATUS.DEADEND, STATUS.NONEXISTENT):
                    neighbor.set_blocked(reverse_heading, value)

    def is_blocked(self, x, y, heading) -> bool:
        inter = self.getintersection(x, y)
        return inter.is_blocked(heading)

    def clear_blockages(self):
        """Clear all blockages from all intersections in the map."""
        for intersection in self.intersections.values():
            for heading in range(8):
                intersection.set_blocked(heading, False)
        print("All blockages have been cleared from the map.")

    def get_cost(self, x, y):
        """
        Return the cost to reach intersection (x, y) after running dijkstra.
        If the intersection does not exist, return float('inf').
        """
        inter = self.intersections.get((x, y))
        if inter is not None:
            return inter.cost
        else:
            return float('inf')






