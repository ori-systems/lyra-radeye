#include "radeye/radeye.h"

//Basic node to run radeye sensor

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RadEye");
	ros::NodeHandle n("~");
    ros::Rate r(1); //The sensor only published as 1Hz

    RadEyeSensor Sensor(n);
    
    while(ros::ok())
    {
        
        Sensor.run();
        r.sleep();
    }


}
