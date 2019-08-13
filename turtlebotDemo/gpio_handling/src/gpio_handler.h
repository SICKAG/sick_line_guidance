#include "ros/ros.h"
#include "custom_messages/gpio.h"
#include "gpio_handling/gpio_set_config.h"
#include "gpio_handling/gpio_get_config.h"
#include "gpio_handling/gpio_set_pin.h"
#include "gpio_handling/gpio_get_pin.h"
#include <string>
#include <iostream>

// messages for publish and subscribing for communication to openCR-board
custom_messages::gpio gpio_set_msg;
custom_messages::gpio gpio_get_msg;

// function Declaration
void gpioCallback(const custom_messages::gpioConstPtr &msg);
bool gpio_set_config(gpio_handling::gpio_set_config::Request &req, gpio_handling::gpio_set_config::Response &res);
bool gpio_get_config(gpio_handling::gpio_get_config::Request &req, gpio_handling::gpio_get_config::Response &res);
bool gpio_set_pin(gpio_handling::gpio_set_pin::Request &req, gpio_handling::gpio_set_pin::Response &res);
bool gpio_get_pin(gpio_handling::gpio_get_pin::Request &req, gpio_handling::gpio_get_pin::Response &res);
