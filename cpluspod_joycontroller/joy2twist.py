import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand

import rich
import time
from rich.console import Console

console = Console()

class joycon_controller(Node):
    def __init__(self):
        super().__init__('joycon_controller')
        self.twist_pub = self.create_publisher(AckermannControlCommand, "/control/command/control_cmd", 100)

        self.indicator_pub = self.create_publisher(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd", 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        
        self.isPublishCmd = False
        self.isConstantSpeed = False
        self.changespeed_time = time.perf_counter()
        self.declare_parameter('constant_speed', 1.0)

        self.declare_parameter('linear_x_offset', 1.0)
        self.declare_parameter('angular_z_offset', 1.0)

        self.linear_x_offset = self.get_parameter('linear_x_offset').value
        self.angular_z_offset = self.get_parameter('angular_z_offset').value
        self.constant_speed = self.get_parameter('constant_speed').value
    def indicator_publish(self, command):
        ticmd = TurnIndicatorsCommand()
        ticmd.command = command
        self.indicator_pub.publish(ticmd)

    def joy_callback(self, msgs):
        speed = self.constant_speed
        steer = msgs.axes[2] * self.angular_z_offset
        if (not self.isConstantSpeed):
            speed = round(msgs.axes[1], 2) * self.linear_x_offset


        ackermann = AckermannControlCommand()
        ackermann.longitudinal.speed =  speed
        ackermann.lateral.steering_tire_angle = steer
        ticmd = TurnIndicatorsCommand()
        
        # stop to publish twist
        # A : Publish
        # B : Not Publish
        # X : Info 
        if (msgs.buttons[0]):
            self.isPublishCmd = True
        elif (msgs.buttons[1]):
            self.isPublishCmd = False
        elif (msgs.buttons[3]):
            self.isConstantSpeed = False
        elif (msgs.buttons[4]):
            self.isConstantSpeed = True
        elif (msgs.axes[7] == 1):
            if (time.perf_counter() - self.changespeed_time > 1.0):
                self.constant_speed += 0.5
                self.changespeed_time = time.perf_counter()
                self.status()
        elif (msgs.axes[7] == -1):
            if (time.perf_counter() - self.changespeed_time > 1.0):
                self.constant_speed -= 0.5
                self.changespeed_time = time.perf_counter()
        elif (msgs.axes[6] == -1):
            self.indicator_publish(2)
        elif (msgs.axes[6] == 1):
            self.indicator_publish(3)
        elif (msgs.buttons[6] == 1):
            self.indicator_publish(1)
        if (self.isPublishCmd):
            self.twist_pub.publish(ackermann)
    
    def status(self):
        console.log("Linear Offset : {}".format(self.linear_x_offset))
        console.log("Angular Offset : {}".format(self.angular_z_offset))
        console.log("Constant Speed : {}".format(self.constant_speed))
        console.log("isPublishCmd : {}".format(self.isPublishCmd))
        console.log("isConstantPublish : {}".format(self.isConstantSpeed))



def main(args=None):
    rclpy.init(args=args)

    joycon_controller_class = joycon_controller()
    rclpy.spin(joycon_controller_class)
    joycon_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
