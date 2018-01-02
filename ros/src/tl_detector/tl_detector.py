#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np
import sys

STATE_COUNT_THRESHOLD = 3

label = ['RED', 'YELLOW', 'GREEN', '', 'UNKNOWN']

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = None
        self.camera_image = None
        self.lights = []
        self.has_image = False
        
        #We thank John Chen's mentioning of this apprach in the slack channel #s_p-system-integrat 
        self.path = rospy.get_param('~model_path')
        self.camera_topic = rospy.get_param('~camera_topic')
        ####
        
        
        self.sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        self.sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        self.sub6 = rospy.Subscriber(self.camera_topic, Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.path)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN

        self.last_wp = -1
        self.state_count = 0
		
	    #constrain the rate at which this node is processed to specified rate; ros::spin() would add unnecessary
        #overhead given this node's purpose (see below)
	    #https://stackoverflow.com/questions/40508651/writing-a-ros-node-with-both-a-publisher-and-subscriber        
        self.updateRate = 5
        self.loop()

    def loop(self):
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown():

        # if following condtions met all required subcription callback functions have run
            if self.pose and self.base_waypoints and self.has_image:
                light_wp, state = self.process_traffic_lights()
                '''
                Publish upcoming red lights at camera frequency.
                Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
                of times till we start using it. Otherwise the previous stable state is
                used.
                '''
                if self.state != state:
                    self.state_count = 0
                    self.state = state
                elif self.state_count >= STATE_COUNT_THRESHOLD:
                    #distinguish among cases (publish (-) values if GREEN or UNKNOWN)
                    if state == TrafficLight.GREEN:
                        light_wp = -light_wp
                    elif state == TrafficLight.UNKNOWN:
                        light_wp = -1
						
                    self.last_wp = light_wp
                    self.upcoming_light_pub.publish(Int32(light_wp))
                    #for debug
                    print ("tl_detector state: ", Int32(light_wp))
                    print ("tl_detector state: ", state)
                    print ("tl_detector state: ", Int32(light_wp))
                    print ("tl_detector state: ", state)
					
                else:
                    self.upcoming_light_pub.publish(Int32(self.last_wp))
				
                self.state_count += 1
		
            rate.sleep()
			
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        if self.base_waypoints is None:
            self.base_waypoints = []
            for wp in msg.waypoints:
                self.base_waypoints.append(wp)
		
        #only do this once since these values don't change
        self.sub2.unregister()
        rospy.loginfo('base_waypoints sub unregistered')
		

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """ Interface with ROS to receive image 

        Args:
            msg (Image): image from car-mounted camera

        """
        rospy.logwarn('image received by tl_detector node')
        self.has_image = True
        self.camera_image = msg

    def get_pose_vector(self, pose):
        return np.asarray([pose.position.x, pose.position.y, pose.position.z], np.float32)

    def get_closest_waypoint(self, pose, waypoints, dist):
        """Identifies the index of the given position within locactions if less than specified distance 
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            locations: ordered list of waypoints (either track waypoints or traffic lights waypoints)
            dist: only for positions w/ distances less than 

        """
        #TODO implement

        index = None
        for i,loc in enumerate(waypoints):
            a = self.get_pose_vector(pose)
            b = self.get_pose_vector(loc.pose.pose)
            temp = np.linalg.norm(a-b)
            if dist > temp:
                dist = temp
                index = i
				
        return index
        


    def get_light_state(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            return False

        # Source:
        #We thank John Chen's mentioning of this apprach in the slack channel #s_p-system-integrat 
        
        # fixing convoluted camera encoding...
        if hasattr(self.camera_image, 'encoding'):
            self.attribute = self.camera_image.encoding
            if self.camera_image.encoding == '8UC3':
                self.camera_image.encoding = "rgb8"
        else:
            self.camera_image.encoding = 'rgb8'
        ####
        
        
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        classification = self.light_classifier.get_classification(cv_image)


        return classification

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # List of positions that correspond to the line to stop in front of for a given intersection
        # stop_line_positions = self.config['stop_line_positions']
           

        #TODO find the closest visible traffic light (if one exists)        
        #***only check for lights within 100 meters***#
        tl_index = self.get_closest_waypoint(self.pose.pose, self.lights, 100)
		
        if tl_index is not None:
            light_wp = self.get_closest_waypoint(self.lights[tl_index].pose.pose, self.base_waypoints,1e10) 	
            state = self.get_light_state()
            return light_wp, state		
        else:
            return -1, TrafficLight.UNKNOWN 

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
