#!/usr/bin/env python
import rospy, gazebo_ros, yaml, rospkg, tf, actionlib

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from etu_simulation.srv import *

rospack = rospkg.RosPack()

class observer:
    def __init__(self):
        rospy.init_node('pose_observer')
        try:
            self.dest_folder = rospy.get_param('pose_observer/poses_folder')
            self.robot = rospy.get_param('pose_observer/robot')
            self.frame = rospy.get_param('pose_observer/frame')
        except KeyError:
            rospy.logerr("pose_observer has crashed: parameters not set")
        self.gazebo_poses = []
        self.amcl_poses = []
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('amcl_pose_server')
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.get_pose = rospy.ServiceProxy('/amcl_pose_server', GetPose)

    def record_point(self):
        while(1):
            print("type cancel to cancel point creation at any prompt\n")
            gazebo_pose = self.get_state(self.robot, self.frame).pose
            amcl_pose = self.get_pose().msg.pose
            while(1):
                pose_name = str(raw_input("Please enter a name for this point: \n"))
                correct = str(raw_input(("is this correct?: " + pose_name + " y/n \n")))
                if(pose_name == 'cancel' or correct == 'cancel'):
                    return 0
                if(correct == "y"):
                    break
            print "point to record: gazebo pose then amcl pose \n"
            print gazebo_pose
            print amcl_pose
            ans = str(raw_input("is this correct y/n? \n"))
            if( ans == 'y'):
                g_data = dict(
                    name = pose_name,
                    position = dict(
                        x = gazebo_pose.position.x,
                        y = gazebo_pose.position.y,
                        z = gazebo_pose.position.z,
                    ),
                    orientation = dict(
                        x = gazebo_pose.orientation.x,
                        y = gazebo_pose.orientation.y,
                        z = gazebo_pose.orientation.z,
                        w = gazebo_pose.orientation.w,
                    ),
                )
                a_data = dict(
                    name = pose_name,
                    position = dict(
                        x = amcl_pose.pose.position.x,
                        y = amcl_pose.pose.position.y,
                        z = amcl_pose.pose.position.z,
                    ),
                    orientation = dict(
                        x = amcl_pose.pose.orientation.x,
                        y = amcl_pose.pose.orientation.y,
                        z = amcl_pose.pose.orientation.z,
                        w = amcl_pose.pose.orientation.w,
                    ),
                    covariance = list(amcl_pose.covariance)
                )
                self.gazebo_poses.append(g_data)
                self.amcl_poses.append(a_data)
                return 1
            elif(ans == 'cancel'):
                return 0

    def writer(self):
        with open(self.dest_folder + "/gazebo_poses.yaml", 'a+') as outfile:
            yaml.dump(self.gazebo_poses, outfile, default_flow_style=False)
        with open(self.dest_folder + "/amcl_poses.yaml", 'a+') as outfile:
            yaml.dump(self.amcl_poses, outfile, default_flow_style=False)

    def recorder(self):
        while(1):
            cmd = str(raw_input("type r to record a point, or e to end recording \n"))
            if cmd == 'e':
                if str(raw_input("are you sure y/n \n")) == 'y':
                    if len(self.gazebo_poses) > 0:
                        self.writer()
                        return 1
                    else:
                        return 0
            elif cmd == "r":
                self.record_point()

if __name__ == "__main__":
    ob = observer()
    ob.recorder()
    rospy.signal_shutdown(0)
