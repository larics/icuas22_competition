#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image
import numpy as np
import pyautogui
import os
from cv_bridge import CvBridge, CvBridgeError
import cv2


class Evaluator:

    def __init__(self):
        rospy.Subscriber('/red/entry_ceiling', String, self.ceilingCallback, queue_size=1)
        rospy.Subscriber('/red/entry_z1', String, self.z1Callback, queue_size=1)
        rospy.Subscriber('/red/entry_z2', String, self.z2Callback, queue_size=1)
        rospy.Subscriber('/red/entry_z3', String, self.z3Callback, queue_size=1)
        rospy.Subscriber('/red/challenge_started', Bool, self.startCallback, queue_size=1)
        self.ball_sub = rospy.Subscriber('/red/uav_magnet/gain', Float32, self.releaseCallback, queue_size=1)
        self.reconstruct_sub = rospy.Subscriber('/red/tag_position_reconstructed', Point, self.reconstructCallback, queue_size=1)
        self.image_sub = rospy.Subscriber('/red/tag_image_annotated', Image, self.imageCallback, queue_size=1)
        self.start_time = None
        self.initialized = False
        self.z2_entry = None
        self.z3_entry = None
        self.last_pose = None
        self.ball_released = False
        self.min_dist = 1e10
        self.tag_loc = np.asarray([float(os.getenv('TILE_X_E')), float(os.getenv('TILE_Y_E')), float(os.getenv('TILE_Z_E'))])
        self.count = 0
        self.finished = False
        self.points_reconstruct = -1
        self.points_accuracy = -1
        self.point_time = -25
        self.z3_reached = False
        rospy.sleep(0.0)
        self.log = open('run.log', 'w')
        self.only_once = False

    def imageCallback(self, msg):
        if self.initialized:
            t_d = rospy.Time.now()-self.start_time
            bridge = CvBridge()
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite('/home/frano/uav_ws/'+str(t_d.to_sec())+'.png', cv2_img)
            rospy.loginfo("[Evaluator] Received annotated image at time %s" % t_d.to_sec())
            self.log.write("[Evaluator] Received annotated image at time %s\n" % t_d.to_sec())
            self.image_sub.unregister()

    def ceilingCallback(self, msg):
        if(self.initialized):
            t_d = rospy.Time.now()-self.start_time
            # rospy.loginfo("[Evaluator] Hit ceiling at time %s" % t_d.to_sec())
        else:
            rospy.loginfo("[Evaluator] Things happened before init")

    def z1Callback(self, msg):
        if(self.initialized):
            t_d = rospy.Time.now()-self.start_time
            rospy.loginfo("[Evaluator] Entered zone 1 at time %s" % t_d.to_sec())
            self.log.write("[Evaluator] Entered zone 1 at time %s\n" % t_d.to_sec())
        else:
            rospy.loginfo("[Evaluator] Things happened before init")

    def z2Callback(self, msg):
        if(self.initialized):
            t_d = rospy.Time.now()-self.start_time
            rospy.loginfo("[Evaluator] Entered zone 2 at time %s" % t_d.to_sec())
            self.log.write("[Evaluator] Entered zone 2 at time %s\n" % t_d.to_sec())
        else:
            rospy.loginfo("[Evaluator] Things happened before init")

    def z3Callback(self, msg):
        if(self.initialized):
            t_d = rospy.Time.now()-self.start_time
            rospy.loginfo("[Evaluator] Entered zone 3 at time %s" % t_d.to_sec())
            self.log.write("[Evaluator] Entered zone 3 at time %s\n" % t_d.to_sec())
            self.z3_reached = True
        else:
            rospy.loginfo("[Evaluator] Things happened before init")

    def startCallback(self, msg):
        rospy.loginfo("[Evaluator] Challenge started, time set to zero")
        self.log.write("[Evaluator] Challenge started, time set to zero\n")
        if not self.initialized:
            self.start_time = rospy.Time.now()
        self.initialized = True

    def releaseCallback(self, msg):
        if(self.initialized):
            t_d = rospy.Time.now()-self.start_time
            rospy.loginfo("[Evaluator] Ball released at time %s" % t_d.to_sec())
            self.log.write("[Evaluator] Ball released at time %s\n" % t_d.to_sec())
            rospy.Subscriber('/red/ball/pose', PoseStamped, self.ballPoseCallback, queue_size=1)
            self.ball_sub.unregister()

    def reconstructCallback(self, msg):
        if(self.initialized):
            t_d = rospy.Time.now()-self.start_time
            rospy.loginfo("[Evaluator] Got reconstruction of the tag at time %s" % t_d.to_sec())
            rospy.loginfo("[Evaluator] Reconstructed tag location %s" % [msg.x, msg.y, msg.z])
            rospy.loginfo("[Evaluator] Real tag location %s" % self.tag_loc)

            dist = np.linalg.norm(np.asarray([msg.x, msg.y, msg.z])-self.tag_loc)
            rospy.loginfo("[Evaluator] Reconstruction error %s" % dist)
            self.points_reconstruct = round(math.exp(-2*float(dist))*50.)/2
            self.log.write("[Evaluator] Got reconstruction of the tag at time %s\n" % t_d.to_sec())
            self.log.write("[Evaluator] Reconstructed tag location %s\n" % [msg.x, msg.y, msg.z])
            self.log.write("[Evaluator] Real tag location %s\n" % self.tag_loc)
            self.log.write("[Evaluator] Reconstruction error %s\n" % dist)

            self.reconstruct_sub.unregister()

    def ballPoseCallback(self, msg):
        if self.finished:
            self.stop_stuff()
            return
        if not self.ball_released:
            self.last_pose = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            self.min_dist = np.linalg.norm(self.last_pose-self.tag_loc)
            self.ball_released = True
            return
        elif (self.initialized):

            currPose = np.asarray([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            dist = np.linalg.norm(currPose - self.tag_loc)
            if dist < self.min_dist:
                self.min_dist = dist
            if msg.pose.position.z<0.35:
                t_d = rospy.Time.now()-self.start_time
                rospy.loginfo("[Evaluator] Ball on the ground at time %s" % t_d.to_sec())
                self.log.write("[Evaluator] Ball on the ground at time %s\n" % t_d.to_sec())
                self.points_accuracy = round(math.exp(-0.5*float(self.min_dist))*70.)/2
                # rospy.loginfo("[Evaluator]Points for launch accuracy %s" % points)
                rospy.loginfo("[Evaluator] Min distance of the ball to the target %s" % self.min_dist)
                self.log.write("[Evaluator] Min distance of the ball to the target %s\n" % self.min_dist)
                self.points_time = round(math.exp(-1*float(t_d.to_sec())/100.)*80.)/2
                # rospy.loginfo("[Evaluator]Points for total time to finish %s" % points)
                
                if self.points_reconstruct >= 0:
                    if self.z3_reached: # reached z3
                        rospy.loginfo("[Evaluator] Points for reconstruction %s" % self.points_reconstruct)
                        rospy.loginfo("[Evaluator] Points for launch accuracy %s" % self.points_accuracy)
                        rospy.loginfo("[Evaluator] Points for time %s" % self.points_time)
                        rospy.loginfo("[Evaluator] Zone 3 reached, received reconstruction, points valid and total %s" % (self.points_time + self.points_reconstruct + self.points_accuracy))
                        self.log.write("[Evaluator] Points for reconstruction %s\n" % self.points_reconstruct)
                        self.log.write("[Evaluator] Points for launch accuracy %s\n" % self.points_accuracy)
                        self.log.write("[Evaluator] Points for time %s\n" % self.points_time)
                        self.log.write("[Evaluator] Zone 3 reached, received reconstruction, run valid, points total %s\n" % (self.points_time + self.points_reconstruct + self.points_accuracy))
                        # self.log.write("[Evaluator] Zone 3 reached, received reconstruction, run valid\n")
                    else:
                        rospy.loginfo("[Evaluator] Zone 3 not reached, no points will be awarded")
                        # rospy.loginfo("[Evaluator] Points for reconstruction %s" % self.points_reconstruct)
                        # rospy.loginfo("[Evaluator] Points for launch accuracy %s" % self.points_accuracy)
                        # rospy.loginfo("[Evaluator] Points for time %s" % self.points_time)
                        self.log.write("[Evaluator] Zone 3 not reached, run invalid\n")
                        # self.log.write("[Evaluator] Points for reconstruction %s\n" % self.points_reconstruct)
                        # self.log.write("[Evaluator] Points for launch accuracy %s\n" % self.points_accuracy)
                        # self.log.write("[Evaluator] Points for time %s\n" % self.points_time)
                else:
                    self.log.write("[Evaluator] No tag position reconstruction reiceved, run invalid\n")
                    rospy.loginfo("[Evaluator] No tag position reconstruction reiceved, run invalid\n")
                self.finished = True

    def stop_stuff(self):
        if not self.only_once:
            t_d = rospy.Time.now() - self.start_time
            rospy.loginfo("[Evaluator] Finished at time %s" % t_d.to_sec())
            rospy.sleep(1)
            if not self.log.closed:
                self.log.write("[Evaluator] Finished at time %s\n" % t_d.to_sec())
                self.log.close()
            pyautogui.hotkey('ctrl', 'b')
            rospy.sleep(1.)
            pyautogui.press('k')
            self.only_once = True


    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.initialized:
                curr_time = rospy.Time.now()
                t_d = curr_time - self.start_time

                if t_d.to_sec() > 150:
                    self.log.write("[Evaluator] Exceeded time limit at time %s\n" % t_d.to_sec())
                    rospy.loginfo("[Evaluator] Exceeded time limit at time %s" % t_d.to_sec())
                    break
                if self.log.closed:
                    break
            r.sleep()
        self.stop_stuff()


if __name__ == '__main__':
    rospy.init_node('evaluator')
    ev = Evaluator()
    ev.run()
