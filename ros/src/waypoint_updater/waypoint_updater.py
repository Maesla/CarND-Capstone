#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # TODO: Add other member variables you need below
        
        self.yaw = None
        self.waypoints = None
        self.update()
                
    def update(self):
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			if self.all_info_ready():
				lane = Lane()
				index_closest_waypoint = self.get_closest_waypoint_index()
				for i in range(LOOKAHEAD_WPS):
					waypoint = self.waypoints.waypoints[index_closest_waypoint+i]
					waypoint.twist.twist.linear.x = 13.7 ## 50 km/h ## TODO, understand this parameter
					lane.waypoints.append(waypoint)
				
				self.final_waypoints_pub.publish(lane)
			rate.sleep()

        #rospy.spin()
        
    def all_info_ready(self):
		return self.yaw is not None and self.waypoints is not None		
    
    def pose_cb(self, msg):
		self.current_pose = msg
		orientation = msg.pose.orientation
		
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		self.yaw = euler[2]
		
    def get_closest_waypoint_index(self):
		waypoints_in_local = self.transform_waypoints_to_local_coordinates()
		index = -1
		min_distance = 1e100
		for i in range(len(waypoints_in_local)):
			x_local = waypoints_in_local[i][0]
			y_local = waypoints_in_local[i][1]
			if (x_local > 0): #is ahead
				dist = x_local*x_local + y_local*y_local #squared length is enough, only for comparing
				if (dist < min_distance):
					min_distance = dist
					index = i
		
		return index
	
    def transform_waypoints_to_local_coordinates(self):
		local_waypoints = []
		for global_waypoint in self.waypoints.waypoints:
			x_global = global_waypoint.pose.pose.position.x
			y_global = global_waypoint.pose.pose.position.y
			
			x_local, y_local = self.trasform_coordinates_to_local(x_global, y_global)
			
			local_waypoints.append((x_local, y_local))
		return local_waypoints
    
    def trasform_coordinates_to_local(self, x_global, y_global):
		x_vehicle = self.current_pose.pose.position.x
		y_vehicle = self.current_pose.pose.position.y
		yaw = self.yaw
		
		x_local_temp = x_global - x_vehicle
		y_local_temp = y_global - y_vehicle
		
		x_local = math.cos(yaw)*x_local_temp + math.sin(yaw)*y_local_temp
		y_local = -math.sin(yaw)*x_local_temp + math.cos(yaw)*y_local_temp
		return x_local, y_local
		
		
    def waypoints_cb(self, waypoints):
		self.waypoints = waypoints
 
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
