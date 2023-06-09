import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

from djitellopy import tello
import cv2
from HandLandmarkModule import handLandmarkDetector

class TelloControlNode(Node):
    def __init__(self):
        super().__init__('tello_control_node')

        # Initialize Tello drone
        self.myTello = tello.Tello()
        self.myTello.connect()
        self.get_logger().info('Tello battery level: %s' % self.myTello.get_battery())
        self.myTello.streamon()

        # Set up camera and other settings
        self.width = 1200
        self.height = 700
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.detector = handLandmarkDetector()

        # Create publishers and subscribers
        self.image_pub = self.create_publisher(Image, 'tello/image', 10)
        self.takeoff_sub = self.create_subscription(Empty, 'tello/takeoff', self.takeoff_callback, 10)
        self.land_sub = self.create_subscription(Empty, 'tello/land', self.land_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'tello/cmd_vel', self.cmd_vel_callback, 10)

    def takeoff_callback(self, msg):
        # Takeoff command received
        self.myTello.takeoff()

    def land_callback(self, msg):
        # Land command received
        self.myTello.land()

    def cmd_vel_callback(self, msg):
        # Velocity command received
        lr = msg.linear.x
        fb = msg.linear.y
        ud = msg.linear.z
        yv = msg.angular.z

        # Send RC control to Tello
        self.myTello.send_rc_control(int(lr), int(fb), int(ud), int(yv))
        self.get_logger().info('Received velocity command: [lr: %s, fb: %s, ud: %s, yv: %s]' % (lr, fb, ud, yv))

    def publish_image(self, img):
        # Convert the OpenCV image to a ROS Image message and publish it
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.encoding = 'bgr8'
        msg.height, msg.width, _ = img.shape
        msg.step = 3 * msg.width
        msg.data = img.flatten().tolist()
        self.image_pub.publish(msg)

    def run(self):
        while rclpy.ok():
            _, img = self.cap.read()

            # Process the image and publish it
            img = self.detector.findHands(img, draw=False)
            fingerLs = self.detector.drawFingerPoint(img)

            # Publish the image
            self.publish_image(img)

            rclpy.spin_once(self)

        self.cap.release()
        self.myTello.streamoff()

def main(args=None):
    rclpy.init(args=args)
    tello_control_node = TelloControlNode()
    tello_control_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
