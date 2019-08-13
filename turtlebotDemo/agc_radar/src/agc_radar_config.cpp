#include "ros/ros.h"
#include "agc_radar/agc_radar_config.h"

int main(int argc, char **argv)	// Node Main Function
{
ros::init(argc, argv, "agc_radar_config"); // Initializes Node Name		
if (argc != 2)				
{
    ROS_INFO("cmd : rosrun own own_client arg0 ");
    ROS_INFO("arg0: Schutzzeit, type: float64\n   arg1: Stopzeit, type: float64");
return 1;
}

char buffer [128];
sprintf(buffer, "%s/agc_radar_config", ros::this_node::getNamespace().c_str());
ros::NodeHandle nh; 	
ros::ServiceClient config_client = nh.serviceClient<agc_radar::agc_radar_config>(buffer); 

// Declares the 'srv' service that uses the 'SrvTutorial' service file
agc_radar::agc_radar_config srv;

// Parameters entered when the node is executed as a service request value are stored at 'a' and 'b'
srv.request.Schutzzeit = atoll(argv[1]);
srv.request.Stopzeit = atoll(argv[2]);
config_client.call(srv);

    return 0;
}   