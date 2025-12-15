import matplotlib.pyplot as plt
from enum import Enum
import time
import pickle
import os


class STATUS(Enum):
    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    DEADEND = 3
    CONNECTED = 4

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

class Intersection:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.streets = [STATUS.UNKNOWN for i in range(8)]
        self.cost = float('inf')
        self.direction = None

class Map:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.heading = 0
        self.intersections = {}
        
        self.cost = None
        self.direction = None
        self.goal = None
        self.failed_goal = False
        self.has_visited_first_intersection = False

    def pose(self):
        return (self.x, self.y, self.heading)

    def calcmove(self):
        dx, dy = heading_to_delta[self.heading]
        self.x += dx
        self.y += dy

    def calcturn(self, turn_amount):
        self.heading = (self.heading + turn_amount) % 8

    def calcuturn(self):
        self.heading = (self.heading + 4) % 8

    def getintersection(self, x, y, create=True):
        if (x, y) not in self.intersections:
            if create:
                print(f"🧱 Creating intersection at ({x}, {y})")
                self.intersections[(x, y)] = Intersection(x, y)
            else:
                return None
        return self.intersections.get((x, y))


    def setstreet(self, x, y, heading, status):
        inter = self.getintersection(x, y)
        inter.streets[heading] = status

        if status == STATUS.CONNECTED:
            dx, dy = heading_to_delta[heading]
            nx, ny = x + dx, y + dy
            neighbor = self.getintersection(nx, ny, create=False)
            if neighbor:
                neighbor.streets[(heading + 4) % 8] = status
                print(f" ↳ Also updating reverse: ({nx}, {ny}) heading {(heading + 4) % 8}")




    def markturn(self, turn_amount):
        if not self.has_visited_first_intersection:
            return  # Don't mark anything before first intersection
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
                
    def has_intersection(self, x, y):
        return (x, y) in self.intersections


    def update_connection(self):
        old_x, old_y, old_heading = self.x, self.y, self.heading
        self.calcmove()
        new_x, new_y = self.x, self.y

        # Create intersection at new location only if we've visited the first intersection
        if self.has_visited_first_intersection:
            self.setstreet(new_x, new_y, (old_heading + 4) % 8, STATUS.CONNECTED)
        else:
            # Don't call setstreet — just trust that we'll create the intersection later if needed
            pass

        # Only update reverse (where we came from) if we’ve visited a real intersection
        if self.has_visited_first_intersection:
            self.setstreet(old_x, old_y, old_heading, STATUS.CONNECTED)



    def markdeadend(self, behaviors):
        print("Reached end of the street.")

        current_intersection = self.getintersection(self.x, self.y)
        current_intersection.streets[self.heading] = STATUS.DEADEND
                    
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

        dx, dy = heading_vectors[self.heading]
        plt.arrow(self.x, self.y, dx, dy,
                width=0.2, head_width=0.3, head_length=0.1, color='magenta')

        plt.pause(0.001)

    def show(self):
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

        color_map = {
            STATUS.UNKNOWN: 'black',
            STATUS.NONEXISTENT: 'lightgray',
            STATUS.UNEXPLORED: 'blue',
            STATUS.DEADEND: 'red',
            STATUS.CONNECTED: 'green'
        }

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
                dx, dy = heading_vectors[idx]
                status = intersection.streets[idx]
                color = color_map[status]
                plt.plot([ix, ix + dx], [iy, iy + dy], color=color)
        
        # draw optimal path to goal if it exists
        if self.goal is not None:
            current = self.getintersection(self.x, self.y)
            cx, cy = self.x, self.y
            visited = set()

            while current.direction is not None and (cx, cy) != self.goal:
                visited.add((cx, cy))
                heading = current.direction
                dx, dy = heading_vectors[heading]
                nx, ny = cx + dx, cy + dy

                # Draw the segment in a distinct color (e.g., orange)
                plt.plot([cx, nx], [cy, ny], color='orange', linewidth=2.5)

                # Move to next
                cx, cy = int(round(nx)), int(round(ny))
                if (cx, cy) in visited:  # prevent infinite loop on corrupted paths
                    break
                current = self.getintersection(cx, cy)
            

                

        plt.pause(0.001)


    def dijkstra(self, xgoal, ygoal):
        # Reset all previous cost/direction info
        for inter in self.intersections.values():
            inter.cost = float('inf')  # infinity
            inter.direction = None

        self.goal = (xgoal, ygoal)  

        # initialize goal node
        goal = self.getintersection(xgoal, ygoal)
        self.failed_goal = False
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

                dx, dy = heading_to_delta[heading]
                nx, ny = x + dx, y + dy
                neighbor = self.getintersection(nx, ny)

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
        self.failed_goal = False
        for inter in self.intersections.values():
            inter.cost = float('inf')
            inter.direction = None


    # update the Brain to Include Automatic Driving to a Goal
    def step_toward_goal(self, behaviors):
        x, y, h = self.pose()
        inter = self.getintersection(x, y)

        # if the robot is at the goal, clear the goal and return
        if (x, y) == self.goal:
            print("Goal reached!")
            self.cleargoal()
            return

        # if direction is None, no path to goal is known
        direction = inter.direction
        if direction is None:
            if not self.failed_goal:
                print("No known path to goal.")
                self.cleargoal()
                self.failed_goal = True
            return

        # if the robot is facing the goal, move forward
        if direction == h:
            print("Advancing toward goal...")
            result = behaviors.follow_line()
            # if the robot is at an intersection, pull forward
            if result == "intersection":
                has_street = behaviors.pull_forward()
                self.update_connection()
    
        else:
            turn_amt = (direction - h) % 8
            if turn_amt > 4:
                turn_amt -= 8
                
            if turn_amt == 4:
                turn_dir = "right"
                turn_amt = -4
            elif turn_amt == -4:
                turn_dir = "right"
            else:
                turn_dir = "left" if turn_amt > 0 else "right"

            print(f"Turning {turn_dir} by {abs(turn_amt)} steps to reach heading {direction}")
            behaviors.turning_behavior(turn_dir)
            self.calcturn(turn_amt)
            self.markturn(turn_amt)





