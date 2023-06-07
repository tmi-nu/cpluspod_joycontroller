import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from autoware_auto_control_msgs.msg import AckermannControlCommand

import rich
from rich.console import Console

console = Console()

class joycon_controller(Node):
    def __init__(self):
        super().__init__('joycon_controller')
        self.twist_pub = self.create_publisher(AckermannControlCommand, "/control/command/control_cmd", 100)

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        
        self.isPublishCmd = False
        self.isConstantSpeed = False

        self.declare_parameter('constant_speed', 1.0)

        self.declare_parameter('linear_x_offset', 1.0)
        self.declare_parameter('angular_z_offset', 1.0)

        self.linear_x_offset = self.get_parameter('linear_x_offset').get_parameter_value().double_value
        self.angular_z_offset = self.get_parameter('angular_z_offset').get_parameter_value().double_value
        self.constant_speed = self.get_parameter('constant_speed').get_parameter_value().double_value

    def joy_callback(self, msgs):
        speed = self.constant_speed
        steer = msgs.axes[2]
        if (not self.isConstantSpeed):
            speed = msg.axes[1] 


        ackermann = AckermannControlCommand()
        ackermann.longitudinal.speed =  speed * self.linear_x_offset
        ackermann.lateral.steering_tire_angle = steer * self.angular_z_offset
        
        # stop to publish twist
        # A : Publish
        # B : Not Publish
        # X : Info 
        if (msgs.buttons[0]):
            self.isPublishCmd = True
        elif (msgs.buttons[1]):
            self.isPublishCmd = False
        elif (msgs.buttons[3]):
            self.isConstantSpeed = not self.isConstantSpeed
        elif (msgs.axes[7] == 1):
            self.constant_speed += 0.5
            status()
        elif (msgs.axes[7] == -1):
            self.constant_speed -= 0.5
            status()
        elif (msgs.buttons[4]):
            status()
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
