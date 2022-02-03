#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy, math
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Empty
from std_srvs.srv import Empty, EmptyResponse
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
import os

class SpawnBall:

    def __init__(self):
        self.first_pose_received = False
        self.ball_pose_received = False
        self.uav_current_pose = Pose()
        self.ball_pose = Pose()
        self.uav_to_ball_offset = -0.4
        self.uav_reference_pose = Pose()

        self.spawn_ball = rospy.Service('spawn_ball', Empty, self.spawnBallCallback)
        self.delete_model_service = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        rospy.Subscriber('pose', PoseStamped, self.uavPoseCallback, queue_size=1)
        rospy.Subscriber('ball/pose', PoseStamped, self.ballPoseCallback, queue_size=1)

    def uavPoseCallback(self, msg):
        self.uav_current_pose = msg.pose
        self.first_pose_received = True

    def ballPoseCallback(self, msg):
        self.ball_pose = msg.pose
        self.ball_pose_received = True

    def spawnBallCallback(self, req):
        # Spawn ball at UAV position with some offset
        
        while True:
            if self.first_pose_received == True:
                req = DeleteModelRequest()

                # Resolv Gazebo model ball name
                gazebo_ball_name = rospy.get_namespace() + "ball"
                req.model_name = gazebo_ball_name[1:]
                self.delete_model_service.call(req)
                self.ball_pose_received = False

                command = 'roslaunch icuas22_competition spawn_ball.launch x:=' + \
                    str(round(self.uav_current_pose.position.x, 2)) + ' y:=' + \
                    str(round(self.uav_current_pose.position.y, 2)) + ' z:=' + \
                    str(round(self.uav_current_pose.position.z, 2) + self.uav_to_ball_offset)
                os.system(command)

                r = rospy.Rate(1)
                # Wait for ball to get spawned
                while ((not rospy.is_shutdown()) and (self.ball_pose_received == False)):
                    #print("Ball received: ", self.ball_pose_received)
                    r.sleep()
                
                # Wait for 2s to see if the ball is still attached to the UAV
                rospy.sleep(5)
                
                # Check if ball is too far from the UAV
                dx = self.ball_pose.position.x - self.uav_current_pose.position.x
                dy = self.ball_pose.position.y - self.uav_current_pose.position.y
                dz = self.ball_pose.position.z - self.uav_current_pose.position.z
                d = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if d < 2 * abs(self.uav_to_ball_offset):
                    rospy.loginfo("Ball spawned successfully")
                    break

                rospy.logwarn("Failed to spawn ball, trying again")


        return EmptyResponse()

if __name__ == '__main__':

    rospy.init_node('spawn_ball_at_uav')
    sb = SpawnBall()
    rospy.spin()
