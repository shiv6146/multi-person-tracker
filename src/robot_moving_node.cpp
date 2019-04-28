#include <signal.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"


int nb_static = 5;

using namespace std;

class robot_moving_node {
private:

    ros::NodeHandle n;

     // communication with person_detector
    ros::Publisher pub_robot_moving;

    // communication with odometry
    ros::Subscriber sub_odometry;

    geometry_msgs::Point position, not_moving_position;
    float orientation, not_moving_orientation;
    int count;
    bool moving;
    bool new_odom;//to check if new data of odometry is available or not

public:

robot_moving_node() {

    // communication with person_detector
    pub_robot_moving = n.advertise<std_msgs::Bool>("robot_moving", 1);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &robot_moving_node::odomCallback, this);

    moving = 1;
    count = 0;
    not_moving_position.x = 0;
    not_moving_position.y = 0;
    not_moving_orientation = 0;
    new_odom = false;

    ros::Rate r(20);//this node is updated at 20hz

    while (ros::ok()) {
        ros::spinOnce();
        update();
        r.sleep();
    }

}//robot_moving_node

void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    new_odom = true;
    position.x = o->pose.pose.position.x;
    position.y = o->pose.pose.position.y;
    orientation = tf::getYaw(o->pose.pose.orientation);

}//odomCallback

void update() {

    if ( new_odom ) {//we wait for new data of odometry
        new_odom = false;
        if ( ( not_moving_position.x == position.x ) && ( not_moving_position.y == position.y ) && ( not_moving_orientation == orientation ) ) {
            count++;
            if ( ( count == nb_static ) && ( moving ) ) {
                ROS_INFO("robot is not moving");
                moving = false;
            }
        }
        else {
            not_moving_position.x = position.x;
            not_moving_position.y = position.y;
            not_moving_orientation = orientation;
            count = 0;
            if ( !moving ) {
                ROS_INFO("robot is moving");
                moving = true;
            }
        }

        std_msgs::Bool robot_moving_msg;
        robot_moving_msg.data = moving;

        pub_robot_moving.publish(robot_moving_msg);
    }

}//update

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "robot_moving_node");
    ros::NodeHandle n;

    ROS_INFO("(robot_moving_node) check if the robot is moving or not");

    robot_moving_node bsObject;
    ros::spin();

    return 0;

}