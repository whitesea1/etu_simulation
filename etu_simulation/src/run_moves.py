#!/usr/bin/env python
import rospy, yaml, rospkg, tf, actionlib, os, pickle, random
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from std_msgs.msg import Header
from std_srvs.srv import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import *
from etu_simulation.srv import *
from math import sqrt
from copy import deepcopy


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
            self.results_folder = rospy.get_param('simulator/results_folder')
        except KeyError:
            # print("keyerror")
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
        self.get_amcl_pose = rospy.ServiceProxy('/amcl_pose_server', GetPose)
        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        self.send_goal = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.send_goal.wait_for_server(rospy.Duration(60))

        # proxy for resetting gazebo if needed
        rospy.wait_for_service('gazebo/reset_world')
        self.reset = rospy.ServiceProxy('gazebo/reset_world', Empty)
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
        random.seed(rospy.get_time())
        # Let's run some tests
        # rospy.loginfo("got to simulate")
        for t in range(50):
            i = random.choice(range(0,num))
            j = random.choice(range(0,num))
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
                self.clear_costmaps()
                rospy.sleep(2)
                self.start_time = rospy.get_time()
                self.send_goal.send_goal(self.make_goal(self.locations[j]))
                path = []
                while(not self.stopped):
                    path.append(self.get_amcl_pose().msg.pose.pose)
                    rospy.sleep(1)
                data = [self.move_result,self.stop_time - self.start_time, path]
                self.write_data(self.locations[i],self.locations[j],data)
        rospy.signal_shutdown(0)

    def write_data(self,start,stop,data):
        filename = self.results_folder + '/' + start + '_' + stop + '_' + self.id + '.p'
        if os.path.isfile(filename):
            # Results already exists
            results = pickle.load(open(filename,"rb"))
            results.append(data)
            pickle.dump(results,open(filename,'w'))
        else:
            results = [data]
            pickle.dump(results,open(filename,'w'))


    def make_goal(self,location):
        goal = MoveBaseGoal()
        pose = deepcopy(self.a_locations[location])
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
        # rospy.loginfo("got to set_start")
        zero_twist = Twist()
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
            self.publisher.publish(amcl_initial_pose)
            rospy.sleep(1)
            if self.localized(amcl_initial_pose.pose.pose):
                return 0
        return 1

    def localized(self,pose):
        my_pose = self.get_amcl_pose().msg.pose.pose
        distance = (my_pose.position.x - pose.position.x) * (my_pose.position.x - pose.position.x)
        distance = distance + (my_pose.position.y - pose.position.y) * (my_pose.position.y - pose.position.y)
        distance = distance + (my_pose.position.z - pose.position.z) * (my_pose.position.z - pose.position.z)
        distance = sqrt(distance)
        if distance < .75:
            return 1
        else:
            return 0


    # Get the amcl and gazebo poses by the name from the loaded file.
    def _amcl_pose(self,location):
        pose = PoseWithCovariance()
        # print(self.a_locations[location])
        pose.pose.position = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Point',self.a_locations[location]['position'])
        pose.pose.orientation = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Quaternion',self.a_locations[location]['orientation'])
        pose.covariance = self.a_locations[location]['covariance']
        return pose
    def _gazebo_pose(self,location):
        return message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',self.g_locations[location])
if __name__ == "__main__":
    simulator = Simulator()
    rospy.sleep(10)
    simulator.simulate()
    rospy.spin()
