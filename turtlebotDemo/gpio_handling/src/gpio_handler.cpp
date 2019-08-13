#include "gpio_handler.h"

int main(int argc, char **argv)	// node main function
{
    ros::init(argc, argv, "gpio_handler"); // initialize node with given name
    ROS_INFO("Node %s started", ros::this_node::getName().c_str()); // print info to ros command line
    ros::NodeHandle nh; // init node handle

    // init servive servers
    ros::ServiceServer set_config_server = nh.advertiseService("gpio_set_config", gpio_set_config);
    ROS_INFO("gpio set_config_server ready. Call it with: rosservice call %s", set_config_server.getService().c_str());

    ros::ServiceServer get_config_server = nh.advertiseService("gpio_get_config", gpio_get_config);
    ROS_INFO("gpio get_config_server ready. Call it with: rosservice call %s", get_config_server.getService().c_str());

    ros::ServiceServer set_server = nh.advertiseService("gpio_set_pin", gpio_set_pin);
    ROS_INFO("gpio set_server ready. Call it with: rosservice call %s", set_server.getService().c_str());

    ros::ServiceServer get_server = nh.advertiseService("gpio_get_pin", gpio_get_pin);
    ROS_INFO("gpio get_server ready. Call it with: rosservice call %s", get_server.getService().c_str());

    // init publisher and subscriber
    ros::Publisher set_publisher = nh.advertise<custom_messages::gpio>("/gpio_set", 1, false);
    ros::Subscriber get_subscriber = nh.subscribe("/gpio_get", 1, gpioCallback);

    /* gpio-message default values */
    for (int i = 0; i <= 17; i++)
    {
        gpio_set_msg.pin_config[i] = 4; //input pulldown (high impedance)
        gpio_set_msg.pin_value[i] = 0;
        gpio_get_msg.pin_value[i] = 4;
        gpio_get_msg.pin_value[i] = 0;
    }

    ros::Rate loop_rate(200); // publish message rate = 200 Hz

    while(ros::ok())
    {
        ros::spinOnce(); // handle service requests and subscriber callbacks
        set_publisher.publish(gpio_set_msg); // publish messages
        loop_rate.sleep(); // sleep for loop_rate
    }		
    return 0;
}

void gpioCallback(const custom_messages::gpioConstPtr &msg){
    gpio_get_msg = *msg;
}

bool gpio_set_config(gpio_handling::gpio_set_config::Request &req, gpio_handling::gpio_set_config::Response &res){   
    for(uint8_t i = 0; i < req.pinNumber.size(); i++)
    {
        gpio_set_msg.pin_config[req.pinNumber[i]] = req.pinMode[i];
    }
    return true;
}

bool gpio_get_config(gpio_handling::gpio_get_config::Request &req, gpio_handling::gpio_get_config::Response &res){   
    res.pinMode.resize(req.pinNumber.size());
    for(uint8_t i = 0; i < req.pinNumber.size(); i++)
    {
        res.pinMode[i] = gpio_get_msg.pin_config[req.pinNumber[i]];
    }
    return true;
}

bool gpio_set_pin(gpio_handling::gpio_set_pin::Request &req, gpio_handling::gpio_set_pin::Response &res){
    for(uint8_t i = 0; i < req.pinNumber.size(); i++)
    {
        gpio_set_msg.pin_value[req.pinNumber[i]] = req.pinValue[i];
    }
    return true;
}

bool gpio_get_pin(gpio_handling::gpio_get_pin::Request &req, gpio_handling::gpio_get_pin::Response &res){
    res.pinValue.resize(req.pinNumber.size());
    for(uint8_t i = 0; i < req.pinNumber.size(); i++)
    {
        res.pinValue[i] = gpio_get_msg.pin_value[req.pinNumber[i]];
    }
    return true;
}