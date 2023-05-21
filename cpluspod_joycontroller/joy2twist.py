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
        
        self.isPublishTwist = False

        self.declare_parameter('linear_x_offset', 1.0)
        self.declare_parameter('angular_z_offset', 1.0)

        self.linear_x_offset = self.get_parameter('linear_x_offset').get_parameter_value().double_value
        self.angular_z_offset = self.get_parameter('angular_z_offset').get_parameter_value().double_value

    def joy_callback(self, msgs):
        # twist = Twist()
        ackermann = AckermannControlCommand()
        ackermann.longitudinal.speed =  msgs.axes[1] * self.linear_x_offset
        ackermann.lateral.steering_tire_angle = msgs.axes[0] * self.angular_z_offset
        
        # stop to publish twist
        # A : Publish
        # B : Not Publish
        if (msgs.buttons[0]):
            self.isPublishTwist = True
        elif (msgs.buttons[1]):
            self.isPublishTwist = False

        if (self.isPublishTwist):
            self.twist_pub.publish(ackermann)

        # X : Info 
        if (msgs.buttons[3]):
            console.log("linear offset : {}".format(self.linear_x_offset))
            console.log("angular offset : {}".format(self.angular_z_offset))
            console.log("Twist : {}".format('True' if self.isPublishTwist else 'False'))



def main(args=None):
    rclpy.init(args=args)

    joycon_controller_class = joycon_controller()
    rclpy.spin(joycon_controller_class)
    joycon_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
