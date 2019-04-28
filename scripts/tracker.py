#!/usr/bin/python

import rospy
import numpy as np
import random
import math
from scipy.optimize import linear_sum_assignment
import scipy.stats
import scipy.spatial
from geometry_msgs.msg import PointStamped, Point
import copy
import timeit
import message_filters
import sys

from follow_me.msg import Person, PersonArray

from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

from pykalman import KalmanFilter


class DetectedPerson:
    """
    A detected person position.
    """
    def __init__(self, pos_x, pos_y, valid):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.valid = valid


class TrackedPerson:
    """
    A tracked person.
    """
    new_person_id_num = 1

    def __init__(self, x, y, valid):
        self.id_num = TrackedPerson.new_person_id_num
        TrackedPerson.new_person_id_num += 1
        self.colour = (random.random(), random.random(), random.random())
        self.deleted = False
        self.dist_travelled = 0.
        
        # variables for constant velocity Kalman filter with Gaussian acceleration distribution
        # observation vars are given more wightage than the actual motion model itself
        std_process_noise = 0.06666
        delta_t = 1./7.5
        std_pos = std_process_noise
        std_vel = std_process_noise
        std_obs = 0.1
        var_pos = std_pos**2
        var_vel = std_vel**2

        # The observation noise is assumed to be different when updating the Kalman filter than when doing data association
        var_obs_local = std_obs**2 
        self.var_obs = (std_obs + 0.4)**2

        self.filtered_state_means = np.array([x, y, 0, 0])
        self.pos_x = x
        self.pos_y = y
        self.valid = valid
        self.vel_x = 0
        self.vel_y = 0

        # Identity matrix for covariance
        self.filtered_state_covariances = 0.5*np.eye(4) 

        # Constant velocity motion model
        transition_matrix = np.array([[1, 0, delta_t,        0],
                                      [0, 1,       0,  delta_t],
                                      [0, 0,       1,        0],
                                      [0, 0,       0,        1]])

        # Oberservation model. Can observe pos_x and pos_y (unless person is occluded). 
        observation_matrix = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0]])

        transition_covariance = np.array([[var_pos,       0,       0,       0],
                                          [      0, var_pos,       0,       0],
                                          [      0,       0, var_vel,       0],
                                          [      0,       0,       0, var_vel]])

        observation_covariance =  var_obs_local*np.eye(2)

        self.kf = KalmanFilter(
            transition_matrices=transition_matrix,
            observation_matrices=observation_matrix,
            transition_covariance=transition_covariance,
            observation_covariance=observation_covariance,
        )


    def update(self, observations):
        """
        Update our tracked person with new observations
        """
        self.filtered_state_means, self.filtered_state_covariances = (
            self.kf.filter_update(
                self.filtered_state_means,
                self.filtered_state_covariances,
                observations
            )
        )

        # Keep track of the distance it's travelled 
        # We include an "if" structure to exclude small distance changes, 
        # which are likely to have been caused by changes in observation angle
        # or other similar factors, and not due to the object actually moving
        delta_dist_travelled = ((self.pos_x - self.filtered_state_means[0])**2 + (self.pos_y - self.filtered_state_means[1])**2)**(1./2.) 
        if delta_dist_travelled > 0.01: 
            self.dist_travelled += delta_dist_travelled

        self.pos_x = self.filtered_state_means[0]
        self.pos_y = self.filtered_state_means[1]
        self.vel_x = self.filtered_state_means[2]
        self.vel_y = self.filtered_state_means[3]    


class MultiPersonTracker:    
    """
    Tracker for tracking all the persons
    """
    max_cost = 9999999

    def __init__(self):
        self.persons_tracked = []
        self.prev_track_marker_id = 0
        self.prev_person_marker_id = 0
        random.seed(1) 

        self.max_std = 0.9

        self.max_diameter = 0.95
        self.mahalanobis_dist_gate = scipy.stats.norm.ppf(1.0 - (1.0-self.max_diameter)/2., 0, 1.0)
        self.max_cov = self.max_std**2

    	# ROS publishers
        self.marker_pub = rospy.Publisher('track_persons', Marker, queue_size=300)

        # ROS subscribers         
        self.detected_persons_sub = rospy.Subscriber('detected_persons', PersonArray, self.detected_persons_callback)

        rospy.spin() # So the node doesn't immediately shut down

        
    def match_with_nearest_neighbour(self, persons_tracked, persons_detected):
        """
        Match detected persons to existing tracks if any using the min mahalanobis dist threshold and based on
        nearest point in all the detected ones using munkres matching algorithm
        """
        matched_tracks = {}

        # Populate match_dist matrix of mahalanobis_dist between every detection and every track
        match_dist = [] # matrix of probability of matching between all people and all detections.   
        eligible_detections = [] # Only include detections in match_dist matrix if they're in range of at least one track to speed up munkres
        for detect in persons_detected: 
            at_least_one_track_in_range = False
            new_row = []
            for track in persons_tracked:
                # Use mahalanobis dist to do matching
                cov = track.filtered_state_covariances[0][0] + track.var_obs # cov_xx == cov_yy == cov
                mahalanobis_dist = math.sqrt(((detect.pos_x-track.pos_x)**2 + (detect.pos_y-track.pos_y)**2)/cov) # = scipy.spatial.distance.mahalanobis(u,v,inv_cov)**2
                if mahalanobis_dist < self.mahalanobis_dist_gate:
                    cost = mahalanobis_dist
                    at_least_one_track_in_range = True
                else:
                    cost = self.max_cost

                new_row.append(cost)                    
            # If the detection is within range of at least one track, add it as an eligable detection in the munkres matching 
            if at_least_one_track_in_range: 
                match_dist.append(new_row)
                eligible_detections.append(detect)

        # Run munkres on match_dist to get the lowest cost assignment
        if match_dist:
            elig_detect_indexes, track_indexes = linear_sum_assignment(match_dist)
            print(elig_detect_indexes)
            print(track_indexes)
            for elig_detect_idx, track_idx in zip(elig_detect_indexes, track_indexes):
                if match_dist[elig_detect_idx][track_idx] < self.mahalanobis_dist_gate:
                    detect = eligible_detections[elig_detect_idx]
                    track = persons_tracked[track_idx]
                    matched_tracks[track] = detect

        return matched_tracks

      
    def detected_persons_callback(self, detected_persons_msg):    
        """
        Callback for every time detect_persons publishes new sets of detected persons positions. 
        It tries to match the newly detected persons with currently tracked persons from previous frames.
        """
        detected_persons = []
        detected_persons_set = set()
        for idx, person in enumerate(detected_persons_msg.persons):
            print("Person id: " + str(idx) + " Valid = " + str(person.valid) + " | X = " + str(person.position.x) + " Y = " + str(person.position.y))
            new_detected_person = DetectedPerson(
                person.position.x, 
                person.position.y,
                person.valid
            )
            detected_persons.append(new_detected_person)
            detected_persons_set.add(new_detected_person)  
      
		# Propogate existing tracks
        propogated = copy.deepcopy(self.persons_tracked)

        # Match detected objects to existing tracks
        matched_tracks = self.match_with_nearest_neighbour(propogated, detected_persons)

        # Update all tracks with new oberservations 
        tracks_to_delete = set()   
        for idx, track in enumerate(self.persons_tracked):
            propogated_track = propogated[idx] # Get the corresponding propogated track

            if propogated_track in matched_tracks:
                # Found a match for this non-person track
                matched_detection = matched_tracks[propogated_track]
            else:
                matched_detection = None  

            if matched_detection:
                observations = np.array([matched_detection.pos_x, 
                                         matched_detection.pos_y])
                
            else: # propogated_track not matched to a detection
                # don't provide a measurement update for Kalman filter 
                # so send it a masked_array for its observations
                observations = np.ma.masked_array(np.array([0, 0]), mask=[1,1]) 
                        
            # Input observations to Kalman filter
            track.update(observations)

            # Check track for deletion because covariance is too large
            cov = track.filtered_state_covariances[0][0] + track.var_obs # cov_xx == cov_yy == cov
            if cov > self.max_cov:
                tracks_to_delete.add(track)
                # rospy.loginfo("deleting because unseen for %.2f", (now - track.last_seen).to_sec())

        # Delete tracks that have been set for deletion
        for track in tracks_to_delete:
            track.deleted = True
            self.persons_tracked.remove(track)
            
        # If detections were not matched, create a new track  
        for detect in detected_persons:
            if not detect in matched_tracks.values():
                self.persons_tracked.append(TrackedPerson(detect.pos_x, detect.pos_y, detect.valid))
        

        # Publish to rviz
        self.publish_tracked_people()


    def publish_tracked_people(self):
        """
        Publish markers of tracked people to Rviz
        """
        marker_id = 0

        for person in self.persons_tracked:
            if person.valid: # Only publish people who are valid
                ps = PointStamped()
                ps.header.frame_id = "laser"
                ps.point.x = person.pos_x
                ps.point.y = person.pos_y

                # publish rviz markers       
                # Cylinder for body 
                marker = Marker()
                marker.header.frame_id = "laser"
                marker.ns = "People_tracked"
                marker.color.r = person.colour[0]
                marker.color.g = person.colour[1]
                marker.color.b = person.colour[2]          
                marker.color.a = 1.0
                marker.pose.position.x = ps.point.x 
                marker.pose.position.y = ps.point.y
                marker.id = marker_id 
                marker_id += 1
                marker.type = Marker.CYLINDER
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 1.2
                marker.pose.position.z = 0.8
                self.marker_pub.publish(marker)  

                # Sphere for head shape                        
                marker.type = Marker.SPHERE
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2                
                marker.pose.position.z = 1.5
                marker.id = marker_id 
                marker_id += 1                        
                self.marker_pub.publish(marker)     

                # Text showing person's ID number 
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.TEXT_VIEW_FACING
                marker.text = str(person.id_num)
                marker.scale.z = 0.2         
                marker.pose.position.z = 1.7
                self.marker_pub.publish(marker)

                # Arrow pointing in direction they're facing with magnitude proportional to speed
                marker.color.r = person.colour[0]
                marker.color.g = person.colour[1]
                marker.color.b = person.colour[2]          
                marker.color.a = 1.0
                start_point = Point()
                end_point = Point()
                start_point.x = marker.pose.position.x 
                start_point.y = marker.pose.position.y 
                end_point.x = start_point.x + 0.5*person.vel_x
                end_point.y = start_point.y + 0.5*person.vel_y
                marker.pose.position.x = 0.
                marker.pose.position.y = 0.
                marker.pose.position.z = 0.1
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.ARROW
                marker.points.append(start_point)
                marker.points.append(end_point)
                marker.scale.x = 0.05
                marker.scale.y = 0.1
                marker.scale.z = 0.2
                self.marker_pub.publish(marker)                           

                # <self.max_diameter>% bounds of person's position as a circle:
                cov = person.filtered_state_covariances[0][0] + person.var_obs # cov_xx == cov_yy == cov
                std = cov**(1./2.)
                gate_dist_euclid = scipy.stats.norm.ppf(1.0 - (1.0-self.max_diameter)/2., 0, std)
                marker.pose.position.x = ps.point.x 
                marker.pose.position.y = ps.point.y                    
                marker.type = Marker.SPHERE
                marker.scale.x = 2*gate_dist_euclid
                marker.scale.y = 2*gate_dist_euclid
                marker.scale.z = 0.01   
                marker.color.r = person.colour[0]
                marker.color.g = person.colour[1]
                marker.color.b = person.colour[2]            
                marker.color.a = 0.5
                marker.pose.position.z = 0.0
                marker.id = marker_id 
                marker_id += 1                    
                self.marker_pub.publish(marker)                

        # Clear previously published people markers
        for m_id in xrange(marker_id, self.prev_person_marker_id):
            marker = Marker()
            marker.header.frame_id = "laser"
            marker.ns = "People_tracked"
            marker.id = m_id
            marker.action = marker.DELETE   
            self.marker_pub.publish(marker)
        self.prev_person_marker_id = marker_id          


if __name__ == '__main__':
    rospy.init_node('moving_persons_tracker', anonymous=True)
    mpt = MultiPersonTracker()