from sentry_communication.communication import RobotPositionMessage, Serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# MCB uses UART1 at 115200 baud (same as sensor_publisher)
serial = Serial("/dev/ttyTHS1", 115200)

def sendVelocityCommand(command: list[float]):
    message = RobotPositionMessage(command)
    serial.write(message.createMessage())

class NavSubscriber(Node):
    def __init__(self):
        super().__init__('navsubscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        cmd = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.get_logger().info(f'Sending Command: {cmd}')        
        sendVelocityCommand(cmd)


def main():
    rclpy.init(None)
    nav_messenger = NavSubscriber()

    try:
        rclpy.spin(nav_messenger)    
    except KeyboardInterrupt:
        pass
    finally:
        nav_messenger.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
