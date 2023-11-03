#!/usr/bin/env python3
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_bot_by_seq_goal')
        # point sequence for X and Y. O is placed between two points to seprate them:
        points_seq = [0.3,0.5,0,2.2,0.5,0,1.5,-0.5,0, 0.5,1.3,0,1.5,-1.5,0,-1.5,1.5,0, 0.5,-0.5, 0]
        # yaw angle in deg (beacuse there is no ratotions around x and y axes):
        yaweulerangles_seq = [90,0,180, 45,-45,90, 0]
        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        rospy.loginfo(str(points))
        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1

        #Create MoveBase action client
        self.MoveBase_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server")
        # wait for the action server to come up
        wait_for_action_server = self.MoveBase_client.wait_for_server()
        if not wait_for_action_server:
            rospy.logerr("There is no action server accessible!")
            rospy.signal_shutdown("There is no action server accessible!")
            return
        rospy.loginfo("move_base server is now connected")
        rospy.loginfo("Begin achieving goals.")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.MoveBase_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return


    def movebase_client(self):
    #for pose in pose_seq:   
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.MoveBase_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
