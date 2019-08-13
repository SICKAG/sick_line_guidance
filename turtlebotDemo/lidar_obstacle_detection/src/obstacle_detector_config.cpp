#include "ros/ros.h"
#include "lidar_obstacle_detection/obstacle_detector_config.h"

int main(int argc, char **argv)	// Node Main Function
{
ros::init(argc, argv, "obstacle_detector_config"); // Initializes Node Name		
if (argc != 5)				
{
    ROS_INFO("cmd : rosrun own own_client arg0 arg1 arg2 arg3 arg4");
    ROS_INFO("arg0: angle_min, type: int16 \n arg1: angle_max, type: int16 \n arg2: range_min, type: int16 \n arg3: rang_max, type: int16 \n arg4: obstacle_stop_range, type: int16 \n");
return 1;
}

char buffer [128];
sprintf(buffer, "%s/obstacle_detector_config", ros::this_node::getNamespace().c_str());
ros::NodeHandle nh; 	
ros::ServiceClient config_client = nh.serviceClient<lidar_obstacle_detection::obstacle_detector_config>(buffer); 

// Declares the 'srv' service that uses the 'SrvTutorial' service file
lidar_obstacle_detection::obstacle_detector_config srv;

// Parameters entered when the node is executed as a service request value are stored at 'a' and 'b'
srv.request.range_min = atoll(argv[1]);
srv.request.range_max = atoll(argv[2]);
srv.request.angle_min = atoll(argv[3]);
srv.request.angle_max = atoll(argv[4]);
srv.request.obstacle_stop_range = atoll(argv[5]);
config_client.call(srv);
/*
// Request the service. If the request is accepted, display the response value
if (config_client.call(srv))
{
    ROS_INFO("send srv, srv.Request.in: %s", srv.request.in.c_str());
    ROS_INFO("receive srv, srv.Response.result: %s", srv.response.out.c_str());
}
else
{
ROS_ERROR("Failed to call service own_service");
return 1;
}
*/
    return 0;
}   