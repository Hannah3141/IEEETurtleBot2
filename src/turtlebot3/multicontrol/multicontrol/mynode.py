import pyglet

from pyglet.math import Vec2
import pyglet.input as inp
import threading
import time
import signal

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile
from rclpy.node import Node

LINEAR_CONTROL_EXPONENT = 2.0 # how much to exponentiate the input direction by
ANGULAR_CONTROL_EXPONENT = 1.5 # how much to exponentiate the input direction by

MAX_VEL_LINEAR = 0.12
MAX_VEL_ANGULAR = 1

MAX_DELTAV_LINEAR = 0.002
MAX_DELTAV_ANGULAR = 0.03

def clamp(x, minVal, maxVal):
    if x < minVal: return minVal
    if x > maxVal: return maxVal
    return x

def clamp_accel(current_v, target_v, max_delta):
    if current_v < target_v:
        return min(current_v + max_delta, target_v)
    if current_v > target_v:
        return max(current_v - max_delta, target_v)
    return current_v

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
        print("exiting sigint skipper")


class TurtNode(Node):
    def __init__(self, handler):
        super().__init__('mynode')
        self.handler = handler
        self.qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, "cmd_vel", self.qos)

        
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0

        self.timer = self.create_timer(0.01, self.loop)

        self.shutting_down = False
        #rclpy.get_default_context().on_shutdown(self.shutdown)

    def move(self, linear, angular):
        if not self.shutting_down:
            twist = Twist()
            twist.linear.x = linear
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = angular
            self.pub.publish(twist)

    def shutdown(self):
        self.shutting_down = True
        self.pub.publish(Twist())
        print("node shutdown!")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))
        print("node sleep?")

    def loop(self):
            
        indir = self.handler.get_final_output()
        self.target_linear_velocity = (abs(indir.y) ** LINEAR_CONTROL_EXPONENT) * MAX_VEL_LINEAR
        self.target_angular_velocity = (abs(indir.x) ** ANGULAR_CONTROL_EXPONENT) * MAX_VEL_ANGULAR

        if (indir.y > 0): self.target_linear_velocity *= -1
        if (indir.x > 0): self.target_angular_velocity *= -1
            
        self.control_linear_velocity = clamp_accel(self.control_linear_velocity, self.target_linear_velocity, MAX_DELTAV_LINEAR)
        self.control_angular_velocity = clamp_accel(self.control_angular_velocity, self.target_angular_velocity, MAX_DELTAV_ANGULAR)
        self.move(self.control_linear_velocity, self.control_angular_velocity)

        

# this holds all the input data
class InputHandler:

    def __init__(self):

        self.inputs = dict() # map of controllers to states
        self.running = True
        self.status = 0
        print()

    def on_stick_motion(self, controller, stick, vector):

        if(stick == "leftstick"):
            self.inputs[controller] = vector

    def get_final_output(self):
        #print(self.inputs)
        if len(self.inputs) == 0: return Vec2(0, 0)
        total = Vec2(0, 0)
        for vec in self.inputs.values():
            total += vec
        return total / len(self.inputs)

# this uses the controller manager, which is more abstracted and therefore I dont trust it
def main():
    manager = inp.ControllerManager() # this handles hotplugging
    handler = InputHandler() # this handles controller

    # these will be called when a controller is plugged/unplugged
    # copied from some ros2 thing somewhere
    def on_connect(controller):
        controller.open()
        controller.rumble_play_weak(1.0, 0.1)
        print("\nConnected:", controller)
        handler.inputs[controller] = Vec2(0, 0)
        controller.push_handlers(handler)


    def on_disconnect(controller):
        print("\nDisconnected:", controller)
        handler.inputs.pop(controller)
        controller.remove_handlers(handler)

    def tick_callback(dt):
        if not handler.running: pyglet.app.exit()
        else:
            print("Current Input:", handler.get_final_output())
            handler.status += 1


    
    manager.on_connect = on_connect
    manager.on_disconnect = on_disconnect

    pyglet.clock.schedule_interval(tick_callback, 1) # print the final output every second

    for controller in manager.get_controllers():
        on_connect(controller)

    rclpy.init()
    node = TurtNode(handler)

    
    def finish_callback():
        node.move(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        handler.running = False
        

    print("Ctrl-C to quit")

    with SigintSkipper(finish_callback):
        threading.Thread(target=pyglet.app.run, args=tuple()).start()
        print("Thread started")
        #pyglet.app.run() # runs the pyglet loop that handles input
        rclpy.spin(node)
        print("Pyglet finished")


    print ("done.")

if __name__ == "__main__":
    main()

