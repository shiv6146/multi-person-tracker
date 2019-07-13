Multi person tracker using LaserScanner
=======================================

This is an extension of the initial follow_me behaviour to track multiple moving persons and visualize them on Rviz.


Overview
========


             geometry_msgs/Point                         Person[] persons                        Marker
LaserScanner -------------------> Moving Person Detector ----------------> Moving Person Tracker -----> Rviz
                                                            Marker


Requirements
============
1. ROS
2. Python
3. pip install pykalman
4. rosbag files for LaserScanner data (or) Laser Scanner physical device


Steps to compile and run
========================
1. Place the follow_me folder inside your catkin workspace
2. Run `catkin_make`
3. Run `roscore` in a new terminal
4. In a new terminal, run `rosbag play <bag_file>` if you have it or connect directly to laser scanner
5. In a new terminal, run `rosrun follow_me moving_person_detector_node`
6. In a new terminal, run `rosrun follow_me tracker.py`
7. In a new terminal, run `rviz` and under Markers select `track_persons` option to see the tracking markers

Sample Run
==========

[![Watch the video](https://i.imgur.com/vKb2F1B.png)](https://youtu.be/vt5fpE0bzSY)
