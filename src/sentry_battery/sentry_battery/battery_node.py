import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32

class BatteryNode(Node):
    def __init__(self): # initial function, start when build self
        super().__init__('battery_node')

        # Battery starts at 100%
        self.battery_level = 100

        # latest bullet count
        self.bullet_count = 25

        self.bullets_low_pub = self.create_publisher(Bool, 'bullets_low', 10)
        self.battery_low_pub = self.create_publisher(Bool, 'battery_low', 10)

        # subscriber: gets bullet count from another node
        self.bullet_sub = self.create_subscription(
            Int32,                                      #incoming data type
            'bullet_count',                             #topic name
            self.bullet_callback,                       #when data arrive call this
            10
        )

        # Publisher (String message) text string output is from here
        self.publisher_ = self.create_publisher(
            String,
            'battery_status',
            10
        )
        
        # Timer: calls timer_callback every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Battery node started')

    def bullet_callback(self, msg):
        self.bullet_count = msg.data
        self.get_logger().info(f'Received bullet count: {self.bullet_count}')

    def timer_callback(self):
        # Decrease battery
        self.battery_level -= 1 #why? not read from the battery

        # Create message
        msg = String() # the massage target 
        msg.data = f'Battery level: {self.battery_level}%'
        # Publish message
        self.publisher_.publish(msg)

        # Print warning if low
        if self.battery_level <= 50:
            self.get_logger().warn('LOW BATTERY WARNING')

        # Stop at 0%
        if self.battery_level <= 0:
            self.get_logger().error('Battery depleted')
            self.timer.cancel()

        # Bullet warning
        if self.bullet_count <= 5:
            self.get_logger().warn('LOW BULLETS WARNING')


def main(args=None):
    rclpy.init(args=args)

    node = BatteryNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()