# interface.py
import threading

class Shared:
    def __init__(self):
        self.command = None
        self.goal = None
        self.pose = None
        self.lock = threading.Lock()

        self.robotx = 0
        self.roboty = 0
        self.robotheading = 0
        
    # Method to acquire/gain access to the shared data.
    def acquire(self):
        return self.lock.acquire()

    # Method to release/relinquish access to the shared data.
    def release(self):
        self.lock.release()


def ui(shared):
    print("UI thread started. Enter commands: explore, directed_explore, goal, pause, step, resume, left, right, straight, save, load, pose, show, clear, quit")
    while True:
        cmd = input("Command: ").strip().lower()
        with shared.lock:
            if cmd == "goal":
                try:
                    x = int(input("Enter goal x: "))
                    y = int(input("Enter goal y: "))
                    shared.goal = (x, y)
                    shared.command = "goal"
                except ValueError:
                    print("Invalid goal coordinates.")
            elif cmd == "directed_explore":
                try:
                    x = int(input("Enter target goal x: "))
                    y = int(input("Enter target goal y: "))
                    shared.goal = (x, y)
                    shared.command = "directed_explore"
                except ValueError:
                    print("Invalid goal coordinates.")
            elif cmd == "pose":
                try:
                    x = int(input("x: "))
                    y = int(input("y: "))
                    h = int(input("heading (0-7): "))
                    shared.pose = (x, y, h)
                    shared.command = "pose"
                    print("Pose set.")
                except ValueError:
                    print("Invalid pose.")
            elif cmd in ["explore", "pause", "step", "resume", "left", "right", "straight", "save", "load", "show", "clear", "quit"]:
                shared.command = cmd
                if cmd == "quit":
                    break
            else:
                print("Unknown command.")
