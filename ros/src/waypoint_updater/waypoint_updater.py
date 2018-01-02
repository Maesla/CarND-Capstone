#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0 # Maximum Deceleration value
BRAKE_DISTANCE = 30 # distance to begin slowing down

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.yaw = None
        self.waypoints = None
        self.waypoints_length = None
        self.traffic_light_pos = -1
        self.velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))
        self.update()

    def update(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            if self.all_info_ready():
                lane = Lane()
                index_closest_waypoint = self.get_closest_waypoint_index()
                rospy.logwarn("Current waypoint id is: {0}".format(index_closest_waypoint))
                last_index = index_closest_waypoint + LOOKAHEAD_WPS
                ## To handle point wrapping.
                looped_wp = False;
                if last_index > self.waypoints_length:
                    rospy.loginfo("looping waypoints")
                    waypoints = self.waypoints[index_closest_waypoint:]
                    waypoints += self.waypoints[:last_index-self.waypoints_length]
                    looped_wp = True
                else:
                    waypoints = self.waypoints[index_closest_waypoint:last_index]
                brake_wp, brake = self.check_brake_distance(index_closest_waypoint, last_index, looped_wp)
                for i in range(LOOKAHEAD_WPS):
                    self.set_waypoint_velocity(waypoints, i, self.velocity)
                if brake:
                    self.decelerate(waypoints,0, brake_wp)
                lane.waypoints = waypoints
                self.final_waypoints_pub.publish(lane)
            rate.sleep()

    # rospy.spin()

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
            if (x_local > 0):  # is ahead
                dist = x_local * x_local + y_local * y_local  # squared length is enough, only for comparing
                if (dist < min_distance):
                    min_distance = dist
                    index = i

        return index

    def transform_waypoints_to_local_coordinates(self):
        local_waypoints = []
        for global_waypoint in self.waypoints:
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

        x_local = math.cos(yaw) * x_local_temp + math.sin(yaw) * y_local_temp
        y_local = -math.sin(yaw) * x_local_temp + math.cos(yaw) * y_local_temp
        return x_local, y_local

    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints
        self.waypoints_length = len(self.waypoints)

    def traffic_cb(self, msg):
        rospy.logwarn("Recieved message for waypoint {0}".format(msg.data))
        self.traffic_light_pos = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    ## taken from the waypoint_loader
    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    ## modified from waypoint_loader
    def decelerate(self, waypoints, start_wp, end_wp):
        self.set_waypoint_velocity(waypoints, end_wp, 0.0)
        for i in range(start_wp, end_wp+1):
            dist = self.distance(waypoints, end_wp, i)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            self.set_waypoint_velocity(waypoints, i, min(vel, waypoints[i].twist.twist.linear.x))
        return waypoints

    def check_brake_distance(self, start_index, end_index, looped):
        traffic_index_local = self.traffic_light_pos
        if traffic_index_local == -1:
            return -1, True
        if looped and traffic_index_local < start_index:
            traffic_index_local += self.waypoints_length
        elif traffic_index_local < start_index:
            return -1, True
        traffic_adjusted_index = traffic_index_local - start_index
        return traffic_adjusted_index, traffic_adjusted_index <= BRAKE_DISTANCE


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
