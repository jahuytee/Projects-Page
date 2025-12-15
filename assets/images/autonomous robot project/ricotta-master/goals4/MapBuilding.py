import matplotlib.pyplot as plt
from enum import Enum

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
        self.streets = [STATUS.UNKNOWN for _ in range(8)]

class PoseTracker:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.heading = 0
        self.intersections = {}

    def calcmove(self):
        dx_dy_map = {
            0: (0, 1), 
            1: (-1, 1), 
            2: (-1, 0), 
            3: (-1, -1),
            4: (0, -1), 
            5: (1, -1), 
            6: (1, 0), 
            7: (1, 1)
        }
        dx, dy = dx_dy_map[self.heading]
        self.x += dx
        self.y += dy

    def calcturn(self, turn_amount):
        self.heading = (self.heading + turn_amount) % 8

    def calcuturn(self):
        self.heading = (self.heading + 4) % 8

    def pose(self):
        return (self.x, self.y, self.heading)

    def getintersection(self, x, y):
        if (x, y) not in self.intersections:
            self.intersections[(x, y)] = Intersection(x, y)
        return self.intersections[(x, y)]

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
            # Draw all streets in all directions
            for idx in range(8):
                dx, dy = heading_vectors[idx]
                status = intersection.streets[idx]
                color = color_map[status]
                plt.plot([ix, ix + dx], [iy, iy + dy], color=color)

        # Draw robot's current pose as magenta arrow
        robot_dx, robot_dy = heading_vectors[self.heading]
        plt.arrow(self.x, self.y, robot_dx, robot_dy,
                  width=0.2, head_width=0.3, head_length=0.1, color='magenta')

        plt.pause(0.001)
