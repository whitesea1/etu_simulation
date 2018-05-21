#!/usr/bin/env python
import rospy, yaml, rospkg, tf, actionlib, os, pickle
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import *

rospack = rospkg.RosPack()

class Simulator:
    def __init__(self):
        rospy.init_node('simulator')
        try:
            gazebo_file = rospy.get_param('simulator/gazebo_goals')
            amcl_file = rospy.get_param('simulator/amcl_goals')
            self.robot = rospy.get_param('simulator/robot')
            self.g_frame = rospy.get_param('simulator/gazebo_frame')
            self.a_frame = rospy.get_param('simulator/amcl_frame')
            self.result_folder = rospy.get_param('simulator/results_folder')
        except KeyError:
            rospy.logerr("simlulator has crashed: parameters are not set")

        # Setup some variables we need
        self.start_time = rospy.get_time()
        self.stop_time = rospy.get_time()
        self.stopped = False
        self.move_result = None


        # Get my process id
        self.id = str(os.getpid())

        # Get Service proxies for sending gazebo poses, actions, and getting amcl pose
        rospy.wait_for_service('gazebo/set_model_state')
        self.set_gazebo_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.wait_for_service('gazebo/get_model_state')
        self.get_gazebo_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rospy.wait_for_service('amcl_pose_server')
        self.get_amcl_pose = rospy.ServiceProxy('/amcl_pose_server')

        self.send_goal = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.send_goal.wait_for_server(rospy.Duration(60))

        # proxy for resetting gazebo if needed
        rospy.wait_for_service('gazebo/reset_world')
        self.reset = rospy.ServiceProxy('gazebo/reset_world')
        # Subscribe to the results of moves
        self.move_result_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.callback)

        # Publish initial pose estimates:
        self.publisher = rospy.Publisher('initialpose',PoseWithCovarianceStamped,queue_size=10)

        # Load the lists of locations, gazebo first
        # Gazebo poses yaml file must contain a list of structures with
        ## elements for position (xyz), orientation (quaternion), and name
        g_f = open(gazebo_file,'r')
        g_generator = yaml.load_all(g_f)
        self.g_locations = None
        for thing in g_generator:
            self.g_locations = thing
        self.g_locations = self.listToDict(self.g_locations,key='name')
        # Now amcl locations
        # amcl poses yaml file must contain a list of structures with
        ## elements for position (xyz), orientation (quaternion), covariance (list)
        ## and name
        a_f = open(amcl_file, 'r')
        a_generator = yaml.load_all(a_f)
        self.a_locations = None
        for thing in a_generator:
            self.a_locations = thing
        self.a_locations = self.listToDict(self.a_locations,key='name')
        g_f.close()
        a_f.close()
        # Finally get the list of names of locations
        self.locations = list(self.a_locations.keys())


    def simulate(self):
        num = len(self.locations)
        data = None
        # Let's run some tests
        for t in range(100):
            for i in range(0,num):
                for j in range(0,num):
                    if i == j:
                        continue
                    self.stopped = False
                    # Try to set the starting point and localize
                    attempts = 0
                    localized = True
                    while(self.set_start(self.locations[i])):
                        attempts  = attempts + 1
                        if attempts > 2: # cannot localize here, even after reset
                            localized = False
                            break
                    if not localized:
                        data = [self.locations[i],self.locations[j],'localization_error']
                    else:
                        self.start_time = rospy.get_time()
                        self.send_goal(self.make_goal(self.locations[j]))
                        path = []
                        while(not self.stopped):
                            path.append(self.get_amcl_pose.msg.pose.pose)
                            rospy.sleep(1)
                        data = [self.move_result,self.stop_time - self.start_time, path]
                        self.write_data(locations[i],locations[j],data)

    def write_data(self,start,stop,data):
        if os.path.isfile(self.res)


    def make_goal(self,location):
        goal = MoveBaseGoal()
        pose = self.a_locations[location]
        del pose['covariance']
        goal_pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',pose)
        goal.target_pose.pose = goal_pose
        goal.target_pose.header.frame_id = self.a_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        return goal


    def _reset_world(self):
        self.reset()
        initialpose = PoseWithCovarianceStamped()
        initialpose.pose.position.x = 0
        initialpose.pose.position.y = 0
        initialpose.pose.position.z = 0

        initialpose.pose.orientation.x = 0
        initialpose.pose.orientation.y = 0
        initialpose.pose.orientation.z = 0
        initialpose.pose.orientation.w = 1.0

        initialpose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

        initialpose.header.frame_id = self.a_frame
        initialpose.header.stamp = rospy.Time.now()
        self.publisher.publish(initialpose)
        rospy.sleep(1)

    def listToDict(self,lst,key):
        new_dict = dict((d[key],dict(d)) for (index,d) in enumerate(lst))
        for key in new_dict:
            del new_dict[key]['name']
        return new_dict

    def callback(self,msg):
        self.stop_time = rospy.get_time()
        self.move_result = int(msg.status.status)
        self.stopped = True

    def set_start(self,location):
        gazebo_state = ModelState()
        zero_twist.linear.x = 0.0
        zero_twist.linear.x = 0.0
        zero_twist.linear.z = 0.0
        zero_twist.angular.x = 0.0
        zero_twist.angular.y = 0.0
        zero_twist.angular.z = 0.0
        gazebo_state.model_name = self.robot
        gazebo_state.pose = self._gazebo_pose(location)
        gazebo_state.twist = zero_twist
        gazebo_state.reference_frame = self.g_frame

        amcl_initial_pose = PoseWithCovarianceStamped()
        amcl_initial_pose.pose = self._amcl_pose(location)
        amcl_initial_pose.header.frame_id = self.a_frame
        amcl_initial_pose.header.stamp = rospy.Time.now()
        self.set_gazebo_state(gazebo_state)
        rospy.sleep(1)
        attempts = 0
        while attempts < 10:
            self.publisher(amcl_initial_pose)
            rospy.sleep(1)
            if localized(amcl_initial_pose):
                return 0
        return 1

    # Get the amcl and gazebo poses by the name from the loaded file.
    def _amcl_pose(self,location):
        return message_converter.convert_dictionary_to_ros_message('geometry_msgs/PoseWithCovariance',self.a_locations[location])
    def _gazebo_pose(self,location):
        return message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',self.a_locations[location])
