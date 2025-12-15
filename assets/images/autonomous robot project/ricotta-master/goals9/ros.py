#!/usr/bin/env python3
#
#   ros.py
#
#   Set up a very simple ROS node to listen for goal/explore commands
#   and publish the current pose (position and heading).
#
#   Node:       /PINAME         (this will use your Pi's name)
#
#   Publish:    ~/pose                  geometry_msgs/Pose
#   Subscribe:  ~/goal                  geometry_msgs/Point
#   Subscribe:  ~/explore               std_msgs/Empty
#
import ctypes
import os
import rclpy
import socket
import time
import threading
import json
from std_msgs.msg import String
from std_msgs.msg import UInt32


from math import pi, sin, cos

from rclpy.node                 import Node
from rclpy.time                 import Time, Duration
from geometry_msgs.msg          import Point, Pose
from std_msgs.msg               import Empty
from uithread                   import Shared


#
#   Simple ROS Node Class
#
class ROSNode(Node):
    # Initialization.
    def __init__(self, name, shared):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Save the shared data object.
        self.shared = shared

        # Create the publisher for the pose information.
        self.pub = self.create_publisher(Pose, '~/pose', 10)

        # Then create subscribers for goal and explore commands.
        self.create_subscription(Point, '~/goal',    self.cb_goal,    10)
        self.create_subscription(Empty, '~/explore', self.cb_explore, 10)
        self.create_subscription(String, '~/dicts' , self.cb_dicts, 10)
        self.create_subscription(UInt32, '~/fetch' , self.cb_fetch, 10)


        # Finally create a timer to drive the node.
        self.timer = self.create_timer(1.0, self.cb_timer)

        # Report and return.
        self.get_logger().info("ROS Node '%s' running" % (name))

    # Shutdown.
    def shutdown(self):
        # Destroy the timer and shut down the node.
        self.destroy_timer(self.timer)
        self.destroy_node()


    # Timer callback.  Send the robot's current pose.
    def cb_timer(self):
        # Grab the current x/y and heading from the shared data.
        if self.shared.acquire():
            posx  = self.shared.robotx
            posy  = self.shared.roboty
            head  = self.shared.robotheading
            self.shared.release()

        # Convert the heading into an angle (in radians).
        theta = pi/4 * float(head+2)

        # Populate the ROS message with the data and send.  The
        # orientation is encoded as a quaternion.
        msg = Pose()
        msg.position.x    = float(posx)
        msg.position.y    = float(posy)
        msg.orientation.z = sin(theta/2)
        msg.orientation.w = cos(theta/2)
        #z is the angle of 
        self.pub.publish(msg)


    # Goal command callback.
    def cb_goal(self, msg):
        # Extract the goal coordinates from the message.
        xgoal = msg.x
        ygoal = msg.y

        # Report.
        self.get_logger().info("Received goal command (%d,%d)" % (xgoal,ygoal))

        # Inject the goal command into your robot thread, just as if
        # you had typed the goal command in the UI thread.
        with self.shared.lock:
            self.shared.command = "goal"
            self.shared.goal = (int(xgoal), int(ygoal))

    # Explore command callback.
    def cb_explore(self, msg):
        # Report.
        self.get_logger().info("Received explore command")

        # Inject the explore command into your robot thread, just as
        # if you had typed the explore command in the UI thread.
        with self.shared.lock:
            self.shared.command = "explore"
            self.shared.goal = None
        
    def cb_fetch(self, msg):
        # Extract the prize ID from the message.
        prize = msg.data

        # Report.
        self.get_logger().info("Received fetch command (%d)" % (prize))

        # Place in shared data!
        if self.shared.acquire():
            self.shared.prize_id = prize
            self.shared.release()

    def cb_dicts(self, msg):
        # Grab the raw message data.
        raw_data = json.loads(msg.data)

        # Extract the intersection: prize -> distance dictionary
        raw_dist_dict = raw_data["dist_dict"]
        inter_prize_distance_dict = {
            int(key): {
                int(prize_key): raw_dist_dict[key][prize_key]
                for prize_key in raw_dist_dict[key]
            }
            for key in raw_dist_dict
        }

        # Extract the prize: info dictionary
        raw_info_dict = raw_data["info_dict"]
        prize_info_dict = {
            int(key): raw_info_dict[key] for key in raw_info_dict
        }

        # Report.

        # Place in shared data!
        if self.shared.acquire():
            self.shared.inter_prize_distance_dict = inter_prize_distance_dict
            self.shared.prize_info_dict = prize_info_dict
            self.shared.release()

#   Main ROS Thread Code
#
def runros(shared):
    # Setup network access for ROS on domain #1.
    os.environ['ROS_LOCALHOST_ONLY']='0'
    os.environ['ROS_DOMAIN_ID']='1'

    # Initialize ROS.
    rclpy.init()

    # Instantiate a simple ROS node, named after the host name, and
    # passing the shared data object.
    node = ROSNode(socket.gethostname(), shared)

    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending Run-ROS due to exception: %s" % repr(ex))

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()


######################################################################
#
#   Testing Code
#
#   This executes if run stand-alone.  Similar functionality should be
#   in your existing code.
#
if __name__ == "__main__":
    

    ### Dummy shared data object and run UI function for testing:


    # Define a dummy robot function for testing.
    def runrobot(shared):
        # Just sleep a lot.  You would have the robot code here.
        try:
            while True:
                time.sleep(1)

        # If signaled, break out of the infinite loop and return.
        except BaseException as ex:
            print("Ending Run-Robot due to exception: %s" % repr(ex))


    ### Dummy code for testing:

    # Instantiate the shared data.
    shared = Shared()

    # Run the ROS thread - notice the argument.
    rosthread = threading.Thread(
        name="ROSThread", target=runros, args=(shared,))
    rosthread.start()

    # Run the ROBOT thread.
    runrobot(shared)

    # Send an exception (Keyboard Interrupt) to the ROS thread
    # to finish. Then wait to join.
    print("Interrupting ROS thread...")
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(rosthread.ident),
        ctypes.py_object(KeyboardInterrupt))
    rosthread.join()

    # Nothing to shutdown
    print("Exiting")
