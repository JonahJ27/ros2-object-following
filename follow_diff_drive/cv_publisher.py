import cv2
import numpy as np
import geometry_msgs.msg
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

class CVControlSubPub(Node):

    def __init__(self):
        super().__init__('cv_control_publisher_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/demo_cam/camera1/image_raw', # may need to remove /image or change to /Image
            self.listener_callback,
            10)
        node = rclpy.create_node('teleop_twist')
        self.pub = node.create_publisher(geometry_msgs.msg.Twist, '/demo/cmd_demo', 10)
        self.delta = 5
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        # Load image (probably not correct)
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # specifies the color mapping for the red ball
        lower = np.array([0, 0, 220], dtype = "uint8")
        upper = np.array([15, 15, 255], dtype = "uint8")
        # maps into into the correct form to detect keypoints
        image = cv2.inRange(image, lower, upper)
        image = cv2.bitwise_not(image)

        # Set our filtering parameters
        # Initialize parameter setting using cv2.SimpleBlobDetector
        params = cv2.SimpleBlobDetector_Params()

        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)
     
        # Detect blobs
        keypoints = detector.detect(image)

        # Draw blobs on our image as red circles
        blank = np.zeros((1, 1))
        blobs = cv2.drawKeypoints(image, keypoints, blank, (0, 0, 255),
                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # initializes the twist message
        twist = geometry_msgs.msg.Twist()
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        print(keypoints)
        # if we don't see the ball we turn until we get a keypoint
        if len(keypoints) == 0:
            twist.linear.x = 0.0
            twist.angular.z = 0.8
        else:
            twist.linear.x = 1.0
            x_pos = keypoints[0].pt[0]
            width = image.shape[1]

            twist.angular.z = - (x_pos - width / 2) / width 

            # if width / 2 > x_pos + self.delta:
            #     twist.angular.z = 0.5
            # elif width / 2 < x_pos - self.delta:
            #     twist.angular.z = -0.5
            # else:
            #     twist.angular.z = 0.0

        self.pub.publish(twist)
        print(twist.linear.x, twist.angular.z)





def main(args=None):
    rclpy.init(args=args)

    cv_control_pubsub = CVControlSubPub()

    rclpy.spin(cv_control_pubsub)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv_control_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
