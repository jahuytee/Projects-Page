import time
import traceback
import pigpio
from DriveSystem import DriveSystem
from Sense import LineSensor
from street_behaviors import Behaviors
from AngleSensor import AngleSensor
from MapBuilding import Map, STATUS
import pickle


# funciton for loading map 
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
    except Exception as e:
        print(f"Failed to load map: {e}")
        print("Starting with a blank map instead.")
        return Map()


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

        print("Simple Brain started.")
        
        last_pose = None

        while True:
            map.showwithrobot()
            current_pose= map.pose()
            if current_pose != last_pose:
                print(f"Pose: {map.pose()}")
                last_pose = current_pose

            # naviagation if goal exists
            if map.goal is not None:
                map.step_toward_goal(behaviors)
                continue


            # user control
            choice = input("Drive (straight, left, right, goto, save, load): ").lower()

            if choice in ("left", "right"):
                turn_amt = behaviors.turning_behavior(choice)
                if choice == "left":
                    drive.drive("spin_r")
                elif choice == "right":
                    drive.drive("spin_l")
                    
                while True:
                    (L,M,R) = sensor.read()
                    if (L,M,R) == (0,1,0):
                        drive.stop()
                        break
                    
                map.calcturn(turn_amt)
                map.markturn(turn_amt)

            elif choice == "straight":
                result = behaviors.follow_line()

                if result == "intersection":
                    has_street = behaviors.pull_forward()
                    map.update_connection()
                    x, y, h = map.pose()

                    if has_street:
                        inter = map.getintersection(x,y)
                        if inter.streets[h] == STATUS.UNKNOWN:
                            map.setstreet(x, y, h, STATUS.UNEXPLORED)
                            print(f"Marking street ahead from ({x},{y}) heading {h} as UNEXPLORED")
                        else:
                            print(f"Street ahead from ({x},{y}) heading {h} already marked as {inter.streets[h].name}")
                    else:
                        map.setstreet(x, y, h, STATUS.NONEXISTENT)
                        print(f"Marking street ahead from ({x},{y}) heading {h} as NONEXISTENT")
                        
                elif result == "end":
                    map.markdeadend(behaviors)
                    continue

                else:
                    print("Unknown line-following result:", result)

            elif choice == "goto":
                try:
                    xgoal = int(input("Enter goal x: "))
                    ygoal = int(input("Enter goal y: "))
                    
                    if not map.has_intersection(xgoal, ygoal):
                        print(f"Intersection ({xgoal}, {ygoal}) does not exist in map.")
                        continue
                        
                    map.dijkstra(xgoal, ygoal)
                    print("Goal set. Navigating...")
                    
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

    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Shutting down.")

    except BaseException as ex:
        print(f"Ending due to exception: {repr(ex)}")
        traceback.print_exc()

    finally:
        drive.stop()
        io.stop()
        print("Robot stopped. Brain shutdown :3")
