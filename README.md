Multi person tracker using LaserScanner
=======================================

This project aims at tracking multiple moving persons to visualize them on Rviz.


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

Demo
====
See demo [here](https://www.loom.com/share/b17e1bf5ea3d4df4950938964182b80e)
