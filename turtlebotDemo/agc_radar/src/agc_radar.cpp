#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "agc_radar/agc_radar_msg.h" // for publishing
#include "agc_radar/agc_radar_config.h" // for service
#include "gpio_handling/gpio_get_config.h"
#include "gpio_handling/gpio_set_config.h"
#include "gpio_handling/gpio_get_pin.h"
#include "gpio_handling/gpio_set_pin.h"
#include <typeinfo>

/*************************************************************************************************
*** Service Client (GPIO-handling)
**************************************************************************************************/
ros::ServiceClient gpio_get_config_client;
ros::ServiceClient gpio_set_config_client;
ros::ServiceClient gpio_get_pin_client;
ros::ServiceClient gpio_set_pin_client;

/*************************************************************************************************
*** Services
**************************************************************************************************/
gpio_handling::gpio_get_config gpio_get_config_srv;
gpio_handling::gpio_set_config gpio_set_config_srv;
gpio_handling::gpio_get_pin gpio_get_pin_srv;
gpio_handling::gpio_set_pin gpio_set_pin_srv;

/*************************************************************************************************
*** Messages
**************************************************************************************************/
agc_radar::agc_radar_msg agc_radar_msg;

bool config_Service_callback(agc_radar::agc_radar_config::Request &req,
                             agc_radar::agc_radar_config::Request &res){
    agc_radar_msg.Schutzzeit = req.Schutzzeit;
    ROS_INFO("set Schutzzeit to: %f", req.Schutzzeit);
    return true;
}

int main(int argc, char **argv)
{
    //defaults
    agc_radar_msg.Schutzzeit = 1;
    agc_radar_msg.Stopzeit = 0.01;

    ros::init(argc, argv, "agc_radar");
    ros::NodeHandle nh;
    ROS_INFO("Node %s started.", ros::this_node::getName().c_str());

    ros::Publisher agc_radar_pub = nh.advertise<agc_radar::agc_radar_msg>("agc_radar", 10);
    ROS_INFO("Publishing on topic %s", agc_radar_pub.getTopic().c_str());
    ros::Rate loop_rate(100);

    ros::ServiceServer config_server = nh.advertiseService("agc_radar_config", config_Service_callback);
    ROS_INFO("Service Server for radar configuration started. Call it with \"rosservice call %s\"", config_server.getService().c_str());

    /* Configure Service Clients */
    gpio_get_config_client = nh.serviceClient<gpio_handling::gpio_get_config>("gpio_get_config");
    gpio_set_config_client = nh.serviceClient<gpio_handling::gpio_set_config>("gpio_set_config");
    gpio_get_pin_client = nh.serviceClient<gpio_handling::gpio_get_pin>("gpio_get_pin");
    gpio_set_pin_client = nh.serviceClient<gpio_handling::gpio_set_pin>("gpio_set_pin");

    uint64_t begin = (uint64_t)ros::Time::now().sec *1e9 + (uint64_t)ros::Time::now().nsec;
    uint64_t begin_stop = (uint64_t)ros::Time::now().sec *1e9 + (uint64_t)ros::Time::now().nsec;
    gpio_get_pin_srv.request.pinNumber = {4};

    while(ros::ok())
    {
        gpio_get_pin_client.call(gpio_get_pin_srv);
        
        if(gpio_get_pin_srv.response.pinValue[0] == true) // radar detected an Object
        {    
            if ((uint64_t)ros::Time::now().sec *1e9 + (uint64_t)ros::Time::now().nsec >= begin + agc_radar_msg.Stopzeit * 1e9)
            {
                agc_radar_msg.obsctacle_stop = true;
            }
            begin_stop = (uint64_t)ros::Time::now().sec *1e9 + (uint64_t)ros::Time::now().nsec;
        }
        else
        {
            if ((uint64_t)ros::Time::now().sec *1e9 + (uint64_t)ros::Time::now().nsec >= begin_stop + agc_radar_msg.Schutzzeit * 1e9)
            {
                agc_radar_msg.obsctacle_stop = false;
            }
            begin = (uint64_t)ros::Time::now().sec *1e9 + (uint64_t)ros::Time::now().nsec;
        }

        /*
        if(gpio_get_msg.pin_value[4] == true) // radar detected an Object
        {
            sum = (0.99*sum + 0.01);
        }
        else
        {
            sum = (0.99*sum - 0.01);
        }

        if (sum < 0)
        {
            sum = 0;
        }
        if( sum > 1)
        {
            sum = 1;
        }

        ROS_INFO("sum: %lf", sum);

        if (sum > 0.5)
        {
            agc_radar_msg.obsctacle_stop = true;
            begin = (uint64_t)ros::Time::now().sec *1e9 + (uint64_t)ros::Time::now().nsec;
        }
        else
        {
            if(  ((uint64_t)ros::Time::now().sec *1e9 + (uint64_t)ros::Time::now().nsec) >= (begin + agc_radar_msg.Schutzzeit)  )
            {
                agc_radar_msg.obsctacle_stop = false;
            }
        }
*/

        

        agc_radar_msg.header.stamp = ros::Time::now();
        agc_radar_pub.publish(agc_radar_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

