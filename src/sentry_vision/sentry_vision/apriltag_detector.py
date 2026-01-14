import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pupil_apriltags
import numpy as np

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        
        # 1. Initialize the CV Bridge (Converts ROS Images <-> OpenCV Images)
        self.bridge = CvBridge()
        
        # 2. Initialize the Detector
        self.get_logger().info('Initializing AprilTag Detector (tag36h11)...')
        self.detector = pupil_apriltags.Detector(families='tag36h11')
        
        # 3. Create Subscribers and Publishers
        # Subscribe to the standard camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
            
        # Publish a debug image so we can see the detection in RViz
        self.debug_publisher = self.create_publisher(Image, '/sentry_vision/debug_image', 10)
        
        self.get_logger().info('AprilTag Detector Node Started. Waiting for images...')

    def listener_callback(self, msg):
        try:
            # A. Convert ROS Image message to OpenCV image
            # 'bgr8' is the standard color format
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # B. Convert to Grayscale for detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # C. Detect Tags
            detections = self.detector.detect(gray)
            
            # D. Log detection and Draw (if any tags found)
            if detections:
                self.get_logger().info(f'Detected {len(detections)} tags')
                
                for tag in detections:
                    # Draw the green box
                    pts = tag.corners.reshape((-1, 1, 2)).astype(int)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
                    
                    # Draw center point
                    cX, cY = int(tag.center[0]), int(tag.center[1])
                    cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            # E. Publish the "Debug" image back to ROS (so we can see it in RViz)
            debug_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.debug_publisher.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()