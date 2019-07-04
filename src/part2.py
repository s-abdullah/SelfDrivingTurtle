# part 1
# roslaunch followbot launch.launch
# part 2
# ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
# part 3
# ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
# part 4
# ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=extra.world

#!/usr/bin/env python
# BEGIN ALL

import rospy, cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
import math, os

class Follower:
    def __init__(self):
        # Set the angular tolerance in degrees converted to radians
        self.angular_tolerance = math.radians(0.001)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("window", cv2.WINDOW_NORMAL)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)

        try:
            self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                               Twist, queue_size=5000)

        except:
            rospy.loginfo("TurtlebotController node terminated.")

        self.twist = Twist()

    def stop(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(3)
        os._exit(0)



    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # color ranges tuned to these particular worlds
        lower_yellow = np.array([10, 10, 10])
        upper_yellow = np.array([255, 255, 250])

        lower_green  = np.array([50, 100, 100])
        upper_green  = np.array([70, 255, 255])

        lower_red    = np.array([0, 100, 60])
        upper_red    = np.array([15, 255, 255])

        lower_blue   = np.array([110, 50, 50])
        upper_blue   = np.array([130, 255, 255])

        # crop only relevant portion of mask
        mask = hsv

        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = 3 * h / 4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # Sense colors only in desired portion of image
        mask_b = cv2.inRange(mask, lower_blue, upper_blue)
        mask_g = cv2.inRange(mask, lower_green, upper_green)
        mask_r = cv2.inRange(mask, lower_red, upper_red)

        if mask_b.any():
            search_right = w / 3
            mask[0:h, 0:search_right]= 0

        elif mask_g.any():
            search_left = 2 * w / 3
            mask[0:h, search_left:w]= 0

        # ensure we stay in the middle of the yellow line
        if mask_r.any():
            i = 0

            while i < 4000:
                i += 1

                mask_y = cv2.inRange(mask, lower_yellow, upper_yellow)
                M = cv2.moments(mask_y)

                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

                    # BEGIN CONTROL
                    err = cx - w / 2

                    self.twist.linear.x = 0.3
                    self.twist.angular.z = -float(err) / 1000
                    self.cmd_vel_pub.publish(self.twist)

            self.stop()
            print "STOPPING "
            self.shutdown()
            self.image_sub.unregister()


        mask_y = cv2.inRange(mask, lower_yellow, upper_yellow)
        M = cv2.moments(mask_y)

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

            # BEGIN CONTROL
            err = cx - w / 2

            self.twist.linear.x = 0.3
            self.twist.angular.z = -float(err) / 1000
            self.cmd_vel_pub.publish(self.twist)

            # END CONTROL
        # cv2.imshow("window", image)
        cv2.imshow("frame", image)

        cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
