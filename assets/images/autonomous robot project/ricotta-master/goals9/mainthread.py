# mainthread.py
import pigpio
import threading
import time
import pickle
import ctypes
from DriveSystem import DriveSystem
from Sense import LineSensor
from AngleSensor import AngleSensor
from street_behaviors import Behaviors
from MapBuilding import Map, STATUS
from uithread import Shared, ui
from proximitysensor import ProximitySensor
from navigation import align_to_road, step_toward_goal, autonomous_step, handle_deadend, directed_exploration
from MapBuilding import prompt_and_load_map
from nfc import NFCSensor
from fetch import fetch

from ros import runros

def brain_main(io):
    drive = DriveSystem(io)
    sensor = LineSensor(io)
    angle = AngleSensor(io)
    proximity_sensor = ProximitySensor(io)
    nfc_sensor = NFCSensor()  # Instantiate a single NFCSensor
    behaviors = Behaviors(io, drive, sensor, angle, proximity_sensor)

    shared = Shared()
    map = prompt_and_load_map()
    if map is None:
            map = Map()

    # Ensure the starting intersection is initialized before any map display
    align_to_road(behaviors, map, shared)
    # Start UI thread
    ui_thread = threading.Thread(target=ui, args=(shared,), daemon=True)
    ui_thread.start()

    # start ros thread
    rosthread = threading.Thread(name="ROSThread", target=runros, args=(shared,))
    rosthread.start()

    exploring = False
    paused = False
    navigating_to_goal = False
    invalid_goal_reported = False
    fetching = False

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
                fetching = False
            elif cmd == "pause":
                paused = True
            elif cmd == "resume":
                paused = False
            elif cmd == "step":
                paused = False
                exploring = True
                navigating_to_goal = False
                invalid_goal_reported = False
                fetching = False
            elif cmd == "fetch":
                exploring = False
                paused = False
                navigating_to_goal = False
                invalid_goal_reported = False
                fetching = True
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
                        fetching = False
                        print("Path found to goal, beginning navigation...")
                    else:
                        print("No valid path found to goal.")
                        navigating_to_goal = False
                else:
                    # Goal doesn't exist in map, use directed exploration
                    print(f"Goal intersection ({goal[0]}, {goal[1]}) not found in map. Starting directed exploration...")
                    exploring = False
                    paused = False
                    navigating_to_goal = True
                    invalid_goal_reported = False
                    fetching = False
            elif cmd in ("left", "right"):
                turn_amt, actual_angle = behaviors.turning_behavior(cmd)
                map.markturn(turn_amt, actual_angle)
                # After turning, check blockage in new heading
                x, y, h = map.pose()
                blocked = behaviors.check_blockage(h)
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
                    blocked = behaviors.check_blockage(h)
                    map.set_blocked(x, y, h, blocked)
                elif result == "end":
                    # Mark dead end and get original pose
                    original_x, original_y, original_heading = map.markdeadend()
                    print("Performing U-turn")
                    # Use the handle_deadend function for all dead end handling
                    handle_deadend(map, behaviors, original_x, original_y, original_heading)

            elif cmd == "save":
                name = input("Filename to save: ")
                with open(name, 'wb') as f:
                    pickle.dump(map, f)
                print("Map saved.")

            elif cmd == "load":
                loaded_map = prompt_and_load_map()
                if loaded_map is not None:
                    map = loaded_map
                    x, y, h = map.pose()
                    with shared.lock:
                        shared.robotx = x
                        shared.roboty = y
                        shared.robotheading = h
                    map.showwithrobot()
                else:
                    print("Map loading cancelled or failed.")

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
            elif cmd == "clear":
                map.clear_blockages()
                map.showwithrobot()  # Show the updated map after clearing blockages

            if not paused:
                if exploring:
                    autonomous_step(map, behaviors)
                elif navigating_to_goal and goal is not None:
                    map.showwithrobot()
                    x, y, current_heading = map.pose()
                    current_inter = map.getintersection(x, y)
                    
                    # Check if we've reached the goal
                    if (x, y) == goal:
                        print("Reached goal!")
                        navigating_to_goal = False
                        map.goal = None
                        invalid_goal_reported = False
                    # If goal exists in map, use normal navigation
                    elif goal in map.intersections:
                        # Check if we have a valid path to the goal
                        if current_inter.direction is None:
                            print("Lost path to goal, replanning...")
                            map.dijkstra(goal[0], goal[1])
                        if map.goal is None:
                            print("No valid path found to goal.")
                            navigating_to_goal = False
                        else:
                            print(f"Moving to goal - Position: ({x}, {y}), Heading: {current_heading}")
                            step_toward_goal(map, behaviors)
                    # If goal doesn't exist in map, use directed exploration
                    else:
                        print(f"Exploring toward goal ({goal[0]}, {goal[1]})")
                        directed_exploration(map, behaviors, goal)
                elif fetching:
                    fetch(nfc_sensor, shared, map, behaviors, treasure=None)
                    map.showwithrobot()

            if cmd == "step":
                paused = True

            x, y, heading = map.pose()
            with shared.lock:
                shared.robotx = x
                shared.roboty = y
                shared.robotheading = heading

            map.showwithrobot()
            time.sleep(0.01)

    finally:
        drive.stop()
        io.stop()
        # Explicitly stop the ROS thread
        ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(rosthread.ident), ctypes.py_object(KeyboardInterrupt))
        rosthread.join()
        nfc_sensor.shutdown()


if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Could not connect to pigpio.")
        exit()

    brain_main(io)
