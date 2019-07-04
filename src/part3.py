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
        self.left = 0
        self.right = 0
        self.frame = 0
        self.cx = 0
        self.cy = 0
        self.decision = 0

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
        rospy.sleep(2)
        os._exit(0)


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # color ranges tuned to these particular worlds
        lower_yellow = np.array([10, 10, 10])
        upper_yellow = np.array([255, 255, 250])

        lower_red    = np.array([0, 100, 60])
        upper_red    = np.array([15, 255, 255])

        # crop only relevant portion of mask
        mask = hsv
        mask_shape = hsv

        # setting underisred aras to 0
        h, w, d = image.shape
        search_w = 2 * w / 4
        search_norm =  w / 8

        search_top = int(3 * h / 5) + 40
        search_bot = int(3 * h / 5) + 140
        mask_shape[0:search_top, 0:w] = 0
        mask_shape[search_bot:h, 0:w] = 0

        lower_red = np.array([0, 100, 60])
        upper_red = np.array([15, 255, 255])

        mask_r = cv2.inRange(mask_shape, lower_red, upper_red)

        if mask_r.any():
            # getiing moments and seeing if its close enough
            MR = cv2.moments(mask_r)

            if MR['m00'] > 0:
                cxr = int(MR['m10'] / MR['m00'])
                cyr = int(MR['m01'] / MR['m00'])
                if (abs(cxr - w/2) <= 175)&(abs(cyr - h/2) <= 175):
                    # print "red things found"
                    cv2.circle(image, (cxr, cyr), 5, (0, 255, 255), -1)
                    im1, cnts, hierarchy = cv2.findContours(mask_r, 2, 1)

                    cnts_img = cnts[0]
                    im1l, cntsl, hierarchyl = cv2.findContours(cv2.imread("../imgs/lBlur.png", 0), 2, 1)
                    im1r, cntsr, hierarchyr = cv2.findContours(cv2.imread("../imgs/rBlur.png", 0), 2, 1)
                    im1s, cntss, hierarchys = cv2.findContours(cv2.imread("../imgs/star1.png", 0), 2, 1)

                    cnt_s = cntss[0]
                    cnt_r = cntsr[0]
                    cnt_l = cntsl[0]

                    ret_l = cv2.matchShapes(cnts_img, cnt_l, 1, 0.0)
                    ret_r = cv2.matchShapes(cnts_img, cnt_r, 1, 0.0)
                    ret_s = cv2.matchShapes(cnts_img, cnt_s, 1, 0.0)

                    if self.frame > 20:
						if ret_s < 3:
							print "star stop"
							self.twist.linear.x = 0.3
							self.twist.angular.z = 0
							self.image_sub.unregister()	
							print "star stopped"
							self.move(0.72)

							self.stop()
							self.shutdown()
						# print ret_r, ret_l, ret_s
                        # print self.right, self.left
						if self.right + self.left <=10:
						    if ret_l < ret_r:
						        # print "TURN LEFT"
						        self.left +=1
						    elif ret_l > ret_r:
						        # print "TURN RIGHT"
						        self.right +=1
						else:
							self.decision = 1
							if self.left > self.right:
							    print "TURN LEFT"
							    # self.left +=1
							    # left
							    mask[0:h, search_w:w] = 0
							else:
							    print "TURN RIGHT"
							    # right
							    mask[0:h, 0:search_w] = 0
							    # self.right +=1
                    else:
                        self.frame+=1

        else:
            self.left = 0
            self.right = 0
            self.frame = 0
            self.decision = 0



        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = 3 * h / 4 + 200
        # print self.cx
        thresh = 90
        if (self.cx == 0)&(self.cy == 0):

            w_left = 3 * w / 6 -200
            w_right = 3 * w / 6 + 200

        else:
            w_left = max(self.cx - thresh, 0)
            w_right = min(self.cx + thresh, 640)


        # cv2.imshow("frameY1", mask)

        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        if self.decision == 0:
	        mask[0:h, 0:w_left] = 0
	        mask[0:h, w_right:w] = 0

        # cv2.imshow("frameY1", mask)


        mask_y = cv2.inRange(mask, lower_yellow, upper_yellow)

        M = cv2.moments(mask_y)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if (self.cx == 0)&(self.cy == 0):
                self.cx = cx
                self.cy = cy 
            else:
                self.cx = cx
                self.cy = cy 

            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

            # BEGIN CONTROL
            err = cx - w / 2

            self.twist.linear.x = 0.3
            self.twist.angular.z = -float(err) / 1000
            self.cmd_vel_pub.publish(self.twist)

            # END CONTROL
        cv2.imshow("frame", image)
        # cv2.imshow("frameY", mask_y)
        # cv2.imshow("frameR", mask_r)

        cv2.waitKey(3)
    def move(self, dist):
        
        # How long should it take us to get there?
        linear_duration = dist / 0.3
        
        # Initialize the movement command
        move_cmd = Twist()
        
        # Set the forward speed
        move_cmd.linear.x = 0.3
        move_cmd.angular.z = -0.18

        
        # Move forward for a time to go the desired distance
        r = rospy.Rate(10)
        ticks = int(linear_duration * 10)
        for t in range(ticks):
            self.cmd_vel_pub.publish(move_cmd)
            r.sleep()

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
