import pyglet

from pyglet.math import Vec2
import pyglet.input as inp
import threading
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

MAX_VEL_LINEAR = 0.22
MIN_VEL_ANGULAR = 2.84

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
        if len(self.inputs) == 0: return 0
        total = Vec2(0, 0)
        for vec in self.inputs.keys():
            total += vec
        return total / len(self.inputs)
            
    
def clamp(x, min, max):
    if x < min: return min
    if x > max: return max
    return x

def loop(handler):


    qos = QosProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0
    try:
        while (True):
            dir = handler.get_final_output()
            target_linear_velocity = dir.y * MAX_VEL_LINEAR
            target_angular_velocity = dir.x * MIN_VEL_ANGULAR

            twist = Twist()
            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity
            pub.publish(twist)
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
            
        


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

    def debug_callback(dt):
        print("Debug Callback!", handler.status)
        handler.status += 1

    
    manager.on_connect = on_connect
    manager.on_disconnect = on_disconnect

    pyglet.clock.schedule_interval(debug_callback, 0.1) # print the final output every second

    for controller in manager.get_controllers():
        on_connect(controller)

    #rclpy.init()


    
    try:
        print("Ctrl-C to quit")
        threading.Thread(target=loop, args=(handler,)).start()
        pyglet.app.run() # runs the pyglet loop that handles input
    except KeyboardInterrupt:
        handler.running = False
        pass
    finally:
        handler.running = False
        #rclpy.shutdown()

    print ("done.")

if __name__ == "__main__":
    main()
