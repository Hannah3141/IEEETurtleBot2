#from https://github.com/ROBOTIS-GIT/turtlebot3/blob/main/turtlebot3_example/turtlebot3_example/turtlebot3_relative_move/turtlebot3_relative_move.py

'''
As the robot drives through each square, mark it as travelled. We will know this based on data from /odom. 
Maybe we will need to relocalize at some point with lidar or just the laser (laser's topic is called /scan and there’s a video for it), -- this is what Kalman filters are for, https://github.com/kimsooyoung/kalman_filter_ros2_tutorial/blob/main/kalman_filter/kalman_filter/kalman_filter_solution.py
especially if the asteroids throw us off course, we’ll need to test it. 
First thing is to drop off our beacon if we have one, and we’ll get the aprilTag ID at the same time if we’re using it. 
Then we head straight for the cave, no matter how many asteroids we do or don’t have 
(ideally we could dump some on the way if we are reliable enough, but i don’t think we will be). 
As soon as we touch that square inside the cave, we can go back and dump whatever geodinium we’ve collected into the close CSC. 
Let’s see how many asteroids our robot can hold, how long we can drive around in the cave, hopefully even outside it, before we have to go dump again. 
Maybe we could do a fancy math equation and calculate how much time we have left vs how far we currently are from the rendezvous pads 
to decide when exactly we need to leave to move the CSCs, but probably by the last 30 seconds (endgame anyone??? lol) 
'''
import math
import os
import sys
import termios

import signal #for sigint

from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

rows, cols = (4, 8)
field = [[0 for i in range(cols)] for j in range(rows)]
field[3][2] = 1 #starting sqaure (there are obviously no astroids here, but whatever)

ros_distro = os.environ.get('ROS_DISTRO', 'humble').lower()
if ros_distro == 'humble':
    from geometry_msgs.msg import Twist as CmdVelMsg
else:
    from geometry_msgs.msg import TwistStamped as CmdVelMsg

terminal_msg = """
TurtleBot3 Relative Move
------------------------------------------------------
From the current pose,
x: goal position x (unit: m)
y: goal position y (unit: m)
theta: goal orientation (range: -180 ~ 180, unit: deg)
------------------------------------------------------
"""
#I copied this class exactly from Michael's multicontrol mynode, idk where he got it from; used to hopefully help stop the idiot at the end
class SigintSkipper:
    def __init__(self, callback):
        self.callback = callback
    def __enter__(self):
        self.got = False
        self.handler_old = signal.signal(signal.SIGINT, self.handler)

    def handler(self, sig, frame):
        self.got = (sig, frame)
        self.callback()

    def __exit__(self, type, value, traceback):
        print('exiting sigint skipper')
class Turtlebot3Path():

    @staticmethod
    def turn(angle, angular_velocity, step): #step is state machine counter i think
        twist = CmdVelMsg()
        angle = math.atan2(math.sin(angle), math.cos(angle)) #I swear this line is doing nothing

        if abs(angle) > 0.05: #this was 0.01 but I made it less accurate
            twist.angular.z = angular_velocity if angle > 0 else -angular_velocity
        else:
            step += 1

        return twist, step

    @staticmethod
    def go_straight(distance, linear_velocity, step):
        twist = CmdVelMsg()

        if distance > 0.01:
            twist.linear.x = linear_velocity  #apparently x is go forward to twist.
        else:
            step += 1

        return twist, step


class Turtlebot3RelativeMove(Node):

    def __init__(self): #python needs to specify self all over the place so that different objects of the same type are distinct
        #scope? idk, here should be good
        def intom(inch): #inch to meter conversion, bc IEEE uses in and turtlebot uses m
            return inch * 0.0254
        def degtorad(deg):
            return deg * 3.141592 / 180

        super().__init__('turtlebot3_relative_move') #its parent/super() is node

        self.segment = 0 #idk when these numbers increase
        #so you see, this obviously drives forward to the center line, rotates left, drives to the beacon mast, rotates right, and drives into the cave
        self.segments = [(intom(12), 0, 0), #drives forward to center line
                         (0, 0, degtorad(-90)), #rotate left
                         (intom(36), 0, 0), #drive to beacon mast
                         (0, 0, 3.14), #rotate right
                         (intom(72), 0, 0), #drive into cave
                         (intom(-20), 0, 0), #drive out of cave (TODO: slower so we don't lose elements?)
                         (0, 0, degtorad(90)), #rotate right to face CSC
                         (intom(12), 0, 0) #drive forward into the CSC
                        ]

        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0 
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.step = 1
        self.get_key_state = False # whether we have new user input to work off of
        self.init_odom_state = False # whether we have new odometry data to work off of

        qos = QoSProfile(depth=10) #basically TCP vs UDP, how much we care about getting all the msgs vs fast communication, more here: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html

        self.cmd_vel_pub = self.create_publisher(CmdVelMsg, 'cmd_vel', qos) #type of data, topic name, that qos nonsense

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos) #message type, topic name, function called when a message is received, qos see above

        self.update_timer = self.create_timer(0.010, self.update_callback) #call that function every 0.01 seconds, i think it does start automatically, idk why it needs to go into a variable

        self.get_logger().info('TurtleBot3 relative move node has been initialised.') #log a message with INFO severity (into the log file i guess, you can find it somewhere in rviz? go ask the ros tutorial)

    #this sets of previous (current) position based on the odometry data (better hope its correct)
    def odom_callback(self, msg): #function called when we get data from odometry subscription
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation) #changes how angles are represented, math smth smth

        self.init_odom_state = True #this tells us whether we should trust the data in last_pose

        self.get_logger().info('current odom data: ' + str(msg.pose.pose.position.x)) #TODO: I think msg.pose.pose.position.x is an absolute position, let's see what format it is and how we can add it to the array
        field[math.floor(msg.pose.pose.position.x / 12)][math.floor(msg.pose.pose.position.y / 12)] = 1 #TODO: where is 0??

    #if we have new odometry data, make a new path
    def update_callback(self): #called from the timer every 0.01 seconds
        if self.init_odom_state: #if we have updated odometry data
            self.generate_path()

    #move in three steps based on user input
    def generate_path(self): #called from update_callback called from the timer every 0.01 seconds
        twist = CmdVelMsg() #the message we will eventually publish to move
        if not self.init_odom_state: #if we don't have new odometry data, no reason to move
            return

        if not self.get_key_state: #if we don't have new user input, go ask for some (and it somehow waits until we get it, i guess stalls here? idk how interrupt works)
            #input_x, input_y, input_theta = self.get_key() #get user input
            #for i in range(len(self.segments)):
            if(self.segment == 1):
                self.get_logger().info('current segment: ' + str(1))
                input_x, input_y, input_theta = self.segments[0]
            elif (self.segment == 2):
                self.get_logger().info('current segment: ' + str(2))
                input_x, input_y, input_theta = self.segments[1]
            elif (self.segment == 3):
                self.get_logger().info('current segment: ' + str(3))
                input_x, input_y, input_theta = self.segments[2]
            elif (self.segment == 4):
                self.get_logger().info('current segment: ' + str(4))
                input_x, input_y, input_theta = self.segments[3]
            else:
                self.generate_stop()
                return

            input_x_global = ( #idk this math exactly
                math.cos(self.last_pose_theta) * input_x - math.sin(self.last_pose_theta) * input_y
            )
            input_y_global = (
                math.sin(self.last_pose_theta) * input_x + math.cos(self.last_pose_theta) * input_y
                )

            self.goal_pose_x = self.last_pose_x + input_x_global
            self.goal_pose_y = self.last_pose_y + input_y_global
            self.goal_pose_theta = self.last_pose_theta + input_theta
            self.get_key_state = True #this indicates if we have new user input to move based on

        else:
            if self.step == 1 and self.segment < len(self.segments): #turn towards goal since we can't drive sideways
                path_theta = math.atan2(
                    self.goal_pose_y - self.last_pose_y,
                    self.goal_pose_x - self.last_pose_x)
                angle = path_theta - self.last_pose_theta
                angular_velocity = 0.3

                self.get_logger().info('segment' + str(self.segment) + ' step1 ' + str(angle)[:4]) #add telemetry

                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            elif self.step == 2 and self.segment < len(self.segments): #drive forward the distance to the goal
                distance = math.sqrt(
                    (self.goal_pose_x - self.last_pose_x)**2 +
                    (self.goal_pose_y - self.last_pose_y)**2)
                linear_velocity = 0.05

                self.get_logger().info('segment' + str(self.segment) + 'step2' + str(distance)[:4]) #add telemetry

                twist, self.step = Turtlebot3Path.go_straight(distance, linear_velocity, self.step)

            elif self.step == 3 and self.segment < len(self.segments): #turn to goal heading
                angle = self.goal_pose_theta - self.last_pose_theta
                angular_velocity = 0.2

                self.get_logger().info('segment' + str(self.segment) + 'step3' + str(angle)[:4]) #add telemetry

                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            elif self.step == 4: #restart, call the last user input invalid now since we already got there
                self.step = 1
                self.get_key_state = False
                self.segment = self.segment + 1

            if ros_distro == 'humble':
                self.cmd_vel_pub.publish(twist)
            else:
                stamped = CmdVelMsg()
                stamped.header.stamp = self.get_clock().now().to_msg()
                stamped.twist = twist
                self.cmd_vel_pub.publish(stamped)

    def generate_stop(self): #Hannah's copy of generate_path() to make this fool stop
        self.get_logger().info('generate stop is running')
        twist = CmdVelMsg() #the message we will eventually publish to move
        if not self.init_odom_state: #if we don't have new odometry data, no reason to move
            return

        if not self.get_key_state: #if we don't have new user input, go ask for some (and it somehow waits until we get it, i guess stalls here? idk how interrupt works)
            #input_x, input_y, input_theta = self.get_key() #get user input

            self.get_key_state = True #this indicates if we have new user input to move based on

        else:
            angle = self.last_pose_theta
            angular_velocity = 0.0

            self.get_logger().info('stop now')
            twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            if ros_distro == 'humble':
                self.cmd_vel_pub.publish(twist)
            else:
                stamped = CmdVelMsg()
                stamped.header.stamp = self.get_clock().now().to_msg()
                stamped.twist = twist
                self.cmd_vel_pub.publish(stamped)


    #gets terminal input, I won't have to worry about this bit XD
    def get_key(self):
        print(terminal_msg) #print directions to terminal
        while True:
            try:
                input_x = float(input('Input x: ')) #input comes from stdin
                break
            except ValueError:
                self.get_logger().info('Invalid input! Please enter a number for x.') #maybe logger prints to stdout somewhere?
        while True:
            try:
                input_y = float(input('Input y: '))
                break
            except ValueError:
                self.get_logger().info('Invalid input! Please enter a number for y.')
        while True:
            try:
                input_theta = float(input('Input theta (deg): '))
                if -180 <= input_theta <= 180:
                    break
                else:
                    self.get_logger().info('Enter a value for theta between -180 and 180.')
            except ValueError:
                self.get_logger().info('Invalid input! Please enter a number for theta.')

        input_theta = numpy.deg2rad(input_theta)

        #something about controlling terminal inferface, idk where it links to/what is affects, doss here: https://www.man7.org/linux/man-pages/man3/termios.3.html
        settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        return input_x, input_y, input_theta

    #converts angle representations, what fun.
    def euler_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args) #just write this, idk or care what exactly it does
    node = Turtlebot3RelativeMove() #this calls init from above

    #this code is all from Michael's multicontrol mynode
    def finish_callback():
        node.generate_stop()
        stop_twist = CmdVelMsg()
        node.cmd_vel_pub.publish(stop_twist) #publish an empty cmdVelMsg to stop
        node.destroy_node()
        rclpy.shutdown()

    with SigintSkipper(finish_callback):
        #threading.Thread(target=pyglet.app.run, args=tuple()).start() #I think this is just for controller input and i can ignore it
        rclpy.spin(node)
'''
    try:
        rclpy.spin(node) #this starts callbacks including the timer i think, docs here: https://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html

    except KeyboardInterrupt or SystemExit:
        node.generate_stop()
        stop_twist = CmdVelMsg()
        node.cmd_vel_pub.publish(stop_twist) #publish an empty cmdVelMsg to stop

        node.destroy_node()
        rclpy.shutdown()

    finally: #idk when this is supposed to run, it doesn't interrupt the other code i think??; it doesn't work for me lol
        node.generate_stop()
        stop_twist = CmdVelMsg()
        node.cmd_vel_pub.publish(stop_twist) #publish an empty cmdVelMsg to stop

        node.destroy_node()
        rclpy.shutdown()
'''

if __name__ == '__main__': #this just lives here in python
    main()
