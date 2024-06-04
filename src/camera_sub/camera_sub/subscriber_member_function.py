# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge
import cv2

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        imgplot = plt.imshow(msg.data)
        plt.show()
        # self.get_logger().info('I heard: "%s"' % msg.data)

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera_left/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        br = CvBridge()
        
        image_frame = br.imgmsg_to_cv2(msg)
        img_gray = cv2.cvtColor(image_frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow("camera", img_gray)   
        cv2.waitKey(1)
        #for i in range(msg.height):
        #    image[i] = msg.data[i*640 : (i+1)*640]
            
        # self.get_logger().info("Size: " +  str(np.size(image[0])))
        # self.get_logger().info("Height: " +  str(msg.height))
        # self.get_logger().info("Width: " +  str(msg.width))
        # imgplot = plt.imshow(image)
        # plt.show()
        #self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CameraSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
