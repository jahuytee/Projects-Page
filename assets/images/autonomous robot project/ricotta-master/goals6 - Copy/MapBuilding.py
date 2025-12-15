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
        self.heading = (self.heading + turn_amount) % 8

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


    def markturn(self, turn_amount):
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

    def update_connection(self):
        dx, dy = self.heading_to_delta[self.heading]
        next_x = self.x + dx
        next_y = self.y + dy
        heading = self.heading

        # Mark the connection *before* moving
        self.setstreet(self.x, self.y, heading, STATUS.CONNECTED)
        self.setstreet(next_x, next_y, (heading + 4) % 8, STATUS.CONNECTED)

        # Then update the robot's internal position
        self.x = next_x
        self.y = next_y

    def markdeadend(self, behaviors):
        print("Reached end of the street.")
        current_intersection = self.getintersection(self.x, self.y)
        current_intersection.streets[self.heading] = STATUS.DEADEND

        # marks unexplored streets connected to deadend as non-existent
        for direction, status in enumerate(current_intersection.streets):
            if status == STATUS.UNEXPLORED:
                current_intersection.streets[direction] = STATUS.NONEXISTENT

        print("Performing U-turn")
        turn_amount = behaviors.turning_behavior("left")
        self.calcturn(turn_amount)

        print("Following line back to intersection")
        result_back = behaviors.follow_line()

        if result_back == "intersection":
            print("Intersection reached again after U-turn")
            behaviors.pull_forward()

            old_x, old_y, old_heading = self.x, self.y, self.heading
            prev_intersection = self.getintersection(old_x, old_y)
            new_intersection = self.getintersection(self.x, self.y)
        else:
            print("Warning: expected to reach intersection but didn't.")


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




