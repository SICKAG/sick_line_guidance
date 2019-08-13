#include "ros/ros.h"
#include "lidar_obstacle_detection/obstacle_detector_msg.h" // for publishing
#include "lidar_obstacle_detection/obstacle_detector_config.h" // for service
#include "sensor_msgs/LaserScan.h" // for subscribing
#include <string>
#include <iostream>
#include <sstream>

lidar_obstacle_detection::obstacle_detector_msg obstacle_detector_msg;

void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    uint16_t angle;
    obstacle_detector_msg.obsctacle_stop = false;
    obstacle_detector_msg.object_distance_min = obstacle_detector_msg.range_max;
    //ROS_INFO("angle_min: %d        angle_max: %d", obstacle_detector_msg.angle_min, obstacle_detector_msg.angle_max);

    for(int16_t i = obstacle_detector_msg.angle_min; i < obstacle_detector_msg.angle_max; i++){
        if(i < 0)
        {
            angle = i+360;
        }
        else
        {
            angle = i;
        }


        if(msg->ranges[angle] > obstacle_detector_msg.range_min && msg->ranges[angle] < obstacle_detector_msg.range_max) // if range value is between range_min and range_max
        {
            //ROS_INFO("in range value at angle: %d      range: %f", angle, msg->ranges[angle]);
            if(msg->ranges[angle] < obstacle_detector_msg.object_distance_min)
            {
                obstacle_detector_msg.object_distance_min = msg->ranges[angle];
            }

            if(msg->ranges[angle] < obstacle_detector_msg.obstacle_stop_range)// object in stop range
            {
                obstacle_detector_msg.obsctacle_stop = true;
            }
        }
    }
}

bool config_Service_callback(lidar_obstacle_detection::obstacle_detector_config::Request &req,
                             lidar_obstacle_detection::obstacle_detector_config::Request &res){
    obstacle_detector_msg.range_min = req.range_min;
    ROS_INFO("set range_min to: %f", req.range_min);
    obstacle_detector_msg.range_max = req.range_max;
    ROS_INFO("set range_max to: %f", req.range_max);
    obstacle_detector_msg.angle_min = req.angle_min;
    ROS_INFO("set angle_min to: %d", req.angle_min);
    obstacle_detector_msg.angle_max = req.angle_max;
    ROS_INFO("set angle_max to: %d", req.angle_max);
    obstacle_detector_msg.obstacle_stop_range = req.obstacle_stop_range;
    ROS_INFO("set obstacle_stop_range to: %f", req.obstacle_stop_range);

    return true;
}

int main(int argc, char **argv){
    //defaults
    obstacle_detector_msg.angle_min = -30;
    obstacle_detector_msg.angle_max = 30;
    obstacle_detector_msg.range_min = 0.2;
    obstacle_detector_msg.range_max = 0.5;
    obstacle_detector_msg.obstacle_stop_range = 0.3;
    obstacle_detector_msg.header.frame_id = "obstacle_detector_frame";

    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle nh;
    ROS_INFO("Node %s started.", ros::this_node::getName().c_str());

    char buffer [128];
    sprintf(buffer, "%s/obstacle_detector_config", ros::this_node::getNamespace().c_str());
    ros::ServiceServer config_server = nh.advertiseService(buffer, config_Service_callback);
    ROS_INFO("Service Server for configuration started. Call it with \"rosservice call %s\"", config_server.getService().c_str());

    sprintf(buffer, "%s/scan", ros::this_node::getNamespace().c_str());
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>(buffer, 1, scan_Callback);
    ROS_INFO("Subscribing topic %s", scan_sub.getTopic().c_str());

    sprintf(buffer, "%s/obstacle", ros::this_node::getNamespace().c_str());
    ros::Publisher obstacle_detector_pub = nh.advertise<lidar_obstacle_detection::obstacle_detector_msg>(buffer, 1);
    ROS_INFO("Publishing topic %s", obstacle_detector_pub.getTopic().c_str());

    ros::Rate loop_rate(100); // Publiziere Nachricht mit 100 Hz

    while(ros::ok())
    {
        obstacle_detector_msg.header.stamp = ros::Time::now();
        obstacle_detector_pub.publish(obstacle_detector_msg);

        ros::spinOnce();
        loop_rate.sleep(); // Sleep for 100 ms
    }
    return 0;
}