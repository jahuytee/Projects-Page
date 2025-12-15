import time
import traceback
import pigpio
import pickle
from DriveSystem import DriveSystem
from Sense import LineSensor
from street_behaviors import Behaviors
from AngleSensor import AngleSensor
from MapBuilding import Map, STATUS, prompt_and_load_map

if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Could not connect to pigpio daemon.")
        exit()

    try:
        drive = DriveSystem(io)
        sensor = LineSensor(io)
        anglesensor = AngleSensor(io)
        behaviors = Behaviors(io, drive, sensor, anglesensor)

        choice = input("Start with blank map or load from file? (blank/load): ").strip().lower()
        map = prompt_and_load_map() if choice == "load" else Map()

        manual_mode = input("Would you like to manually explore the map? (yes/no): ").strip().lower() == "yes"

        print("Simple Brain started.")
        last_pose = None

        if manual_mode:
            print("Attempting initial alignment to road...")

            while True:
                L, M, R = sensor.read()
                if (L, M, R) == (0, 0, 0):
                    print("Off-road — performing U-turn.")
                    turn_amt = behaviors.turning_behavior("left")
                    map.calcturn(turn_amt)

                    result = behaviors.follow_line()
                    if result == "intersection":
                        print("Back at intersection.")
                        behaviors.pull_forward()
                        break
                else:
                    print("On-road — following line forward.")
                    result = behaviors.follow_line()
                    if result == "intersection":
                        print("Reached first intersection.")
                        behaviors.pull_forward()
                        break

            map.getintersection(map.x, map.y)  # Initialize intersection after alignment

        while True:
            map.showwithrobot()
            if manual_mode:
                choice = input("Drive (straight, left, right, goto, save, load): ").lower()

                if choice in ("left", "right"):
                    turn_amt = behaviors.turning_behavior(choice)
                    map.calcturn(turn_amt)
                    map.markturn(turn_amt)

                elif choice == "straight":
                    result = behaviors.follow_line()
                    if result == "intersection":
                        has_street = behaviors.pull_forward()
                        map.update_connection()

                        x, y, h = map.pose()
                        inter = map.intersections.get((x, y))
                        if inter and has_street:
                            if inter.streets[h] == STATUS.UNKNOWN:
                                map.setstreet(x, y, h, STATUS.UNEXPLORED)
                            else:
                                map.setstreet(x, y, h, STATUS.CONNECTED)
                        elif inter:
                            map.setstreet(x, y, h, STATUS.NONEXISTENT)

                        if has_street:
                            for delta in [-1, 1]:
                                diag_heading = (h + delta) % 8
                                if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                    map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                                    print(f"Marking diagonal street at heading {diag_heading} from ({x}, {y}) as NONEXISTENT")

                    elif result == "end":
                        map.markdeadend(behaviors)

                elif choice == "goto":
                    try:
                        xgoal = int(input("Enter goal x: "))
                        ygoal = int(input("Enter goal y: "))
                        if not map.has_intersection(xgoal, ygoal):
                            print(f"Intersection ({xgoal}, {ygoal}) does not exist in map.")
                            continue
                        map.dijkstra(xgoal, ygoal)
                        print("Goal set. Navigating...")
                        while map.goal is not None:
                            map.showwithrobot()
                            map.step_toward_goal(behaviors)
                    except ValueError:
                        print("Invalid goal input.")

                elif choice == "save":
                    filename = input("Enter filename to save (e.g. mymap.pkl): ").strip()
                    with open(filename, 'wb') as file:
                        pickle.dump(map, file)
                    print(f"Map saved to {filename}")

                elif choice == "load":
                    map = prompt_and_load_map()

                else:
                    print("Invalid input. Please enter a valid command.")

                continue

            # Autonomous exploration mode
            if map.goal is not None:
                map.showwithrobot()
                map.step_toward_goal(behaviors)
                continue

            x, y, h = map.pose()
            inter = map.intersections.get((x, y))
            if not inter:
                map.getintersection(x, y)
                inter = map.intersections.get((x, y))

            unknown_dirs = [i for i, status in enumerate(inter.streets) if status == STATUS.UNKNOWN]

            if unknown_dirs:
                best_dir = min(unknown_dirs, key=lambda d: min((d - h) % 8, (h - d) % 8))
                turn_amt = (best_dir - h) % 8
                if turn_amt > 4:
                    turn_amt -= 8
                turn_result = behaviors.turning_behavior("left" if turn_amt > 0 else "right")
                map.calcturn(turn_result)
                map.markturn(turn_result)

                result = behaviors.follow_line()
                if result == "intersection":
                    has_street = behaviors.pull_forward()
                    map.update_connection()

                    x, y, h = map.pose()
                    inter = map.intersections.get((x, y))
                    if inter and has_street:
                        if inter.streets[h] == STATUS.UNKNOWN:
                            map.setstreet(x, y, h, STATUS.UNEXPLORED)
                        else:
                            map.setstreet(x, y, h, STATUS.CONNECTED)
                    elif inter:
                        map.setstreet(x, y, h, STATUS.NONEXISTENT)

                    if has_street:
                        for delta in [-1, 1]:
                            diag_heading = (h + delta) % 8
                            if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                                print(f"Marking diagonal street at heading {diag_heading} from ({x}, {y}) as NONEXISTENT")

                elif result == "end":
                    map.markdeadend(behaviors)
                continue

            unexplored_dirs = [i for i, status in enumerate(inter.streets) if status == STATUS.UNEXPLORED]
            if unexplored_dirs:
                best_dir = min(unexplored_dirs, key=lambda d: min((d - h) % 8, (h - d) % 8))
                turn_amt = (best_dir - h) % 8
                if turn_amt > 4:
                    turn_amt -= 8
                turn_result = behaviors.turning_behavior("left" if turn_amt > 0 else "right")
                map.calcturn(turn_result)
                map.markturn(turn_result)

                result = behaviors.follow_line()
                if result == "intersection":
                    has_street = behaviors.pull_forward()
                    map.update_connection()

                    x, y, h = map.pose()
                    inter = map.intersections.get((x, y))
                    if inter and has_street:
                        if inter.streets[h] == STATUS.UNKNOWN:
                            map.setstreet(x, y, h, STATUS.UNEXPLORED)
                        else:
                            map.setstreet(x, y, h, STATUS.CONNECTED)
                    elif inter:
                        map.setstreet(x, y, h, STATUS.NONEXISTENT)

                    if has_street:
                        for delta in [-1, 1]:
                            diag_heading = (h + delta) % 8
                            if inter.streets[diag_heading] == STATUS.UNKNOWN:
                                map.setstreet(x, y, diag_heading, STATUS.NONEXISTENT)
                                print(f"Marking diagonal street at heading {diag_heading} from ({x}, {y}) as NONEXISTENT")

                elif result == "end":
                    map.markdeadend(behaviors)
                continue

            unexplored_targets = [
                (ix, iy) for (ix, iy), inter in map.intersections.items()
                if STATUS.UNKNOWN in inter.streets or STATUS.UNEXPLORED in inter.streets
            ]
            if unexplored_targets:
                unexplored_targets.sort(key=lambda pos: (map.x - pos[0])**2 + (map.y - pos[1])**2)
                xgoal, ygoal = unexplored_targets[0]
                print(f"Exploration target: ({xgoal}, {ygoal})")
                map.dijkstra(xgoal, ygoal)
                continue

    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Shutting down.")

    except BaseException as ex:
        print(f"Ending due to exception: {repr(ex)}")
        traceback.print_exc()

    finally:
        drive.stop()
        io.stop()
        print("Robot stopped. Brain shutdown :3")
