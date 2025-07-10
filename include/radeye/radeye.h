#include "ros/ros.h"
#include "radeye/RadEyeCom.h"
#include <iostream>
#include <string.h>
#include <radeye_msgs/Radeye.h> 

class RadEyeSensor 
{

    //Class to interface with the thermofisher radeye sensors. 
    //Currently only implemented the G-10 and SX sensors 

    public:
        RadEyeSensor(ros::NodeHandle);
        ~RadEyeSensor();
        std::vector<std::string>readRadEye();
        void decToBinary(int, bool*);
        void run();
        bool connected();

    private:
        
        std::string serial_port_;
        std::string frame_id_;
        std::string unit_name_;
        ros::NodeHandle n_;
        std::vector<ros::Publisher> rad_pub_;
        std::vector<int> radiation_type_;
        RadEyeCom RadEye_;

};

RadEyeSensor::RadEyeSensor(ros::NodeHandle n)
{

    std::string measurement_units,additional_settings;
    this->n_ = n;
    this->n_.param<std::string>("serial_port", this->serial_port_, "/dev/ttyACM0");      
    this->n_.param<std::string>("frame_id", this->frame_id_, "RadEye");
    this->n_.param<std::string>("measurement_units", measurement_units, "");
    this->n_.param<std::string>("additional_settings", additional_settings, "");
    std::cout <<"Connecting to: " << this->serial_port_ << std::endl;
    this->RadEye_.init(this->serial_port_);
    if (this->connected() == 0)
    {
        ROS_ERROR("No sensor cable detected!");
        exit(0);
    }
    this->RadEye_.ActivateCyclicSending();
    this->RadEye_.ClearAccumulatedDose();
    this->unit_name_ = "-1"; // -1 is returned when no sensor is present
    int count = 0;
    
    while (this->unit_name_== "-1")
    {
        this->unit_name_ = std::to_string(this->RadEye_.ReadDeviceSerialNumber());
        usleep(500000);
        std::cout <<  this->unit_name_ << std::endl;
        count++;
        if (count == 20)
        {
            ROS_ERROR("No sensor detected!");      
            exit(0);     
        }
    }
    
    //this->unit_name_ = "rad01";
    std::cout << "Detected unit no: "<< this->unit_name_ << std::endl;
    std::vector<std::string> model;
    count = 0;
    while (model.size() == 0)
    {
        model = this->readRadEye();
        usleep(500000);
        count++;    
        if (count == 20)
        {
            ROS_ERROR("No model detected!");      
            exit(0);     
        }    
    }
    std::cout << "Detected model: " << model[5] << std::endl; 
    if (model[5] == "FH41B2")
    {     
        this->radiation_type_.push_back(4);     //see Radeye message file for type definitions
        if (measurement_units == "")
        {
            this->RadEye_.CommandString("uW25");
        }
        else 
        {
            this->RadEye_.CommandString(measurement_units);
            std::cout <<"Changing Measurement Units" << std::endl;
        }
    }
    else if (model[5] == "SX")
    {
        this->radiation_type_.push_back(2);
        this->radiation_type_.push_back(1);
        if (measurement_units == "")
        {
            this->RadEye_.CommandString("uW40");

        }
        else 
        {
            this->RadEye_.CommandString(measurement_units);
            ROS_WARN("Changing Measurement Units");
        }
        
    }
    else
    {
        // Add extra sensors here
        ROS_ERROR(" Sensor model not implemented! (feel free to add)");      
        exit(0); 
    }



    std::cout <<"additional settings " << additional_settings << std::endl;
    std::string setting = "";
    for (int i = 0; i <= additional_settings.size();i++) 
    {
        if (i == additional_settings.size())
        {
            std::cout << "setting "<< setting <<std::endl;
            this->RadEye_.CommandString(setting);
            setting = "";
        }
        else if (additional_settings[i] == ',')
        {
            std::cout << "setting "<< setting <<std::endl;
            this->RadEye_.CommandString(setting);
            setting = "";
        } else
        {
            setting+= additional_settings[i] ; 
        }
    }


    for(int i = 0; i < this->radiation_type_.size();i++)
    {
        this->rad_pub_.push_back(n.advertise<radeye_msgs::Radeye>("RadEye"+this->unit_name_+"_"+std::to_string(i)+"/data", 10));
        std::cout <<"Publishing on "<< "RadEye"+this->unit_name_+"_"+std::to_string(i)+"/data" << std::endl;
    }


}

RadEyeSensor::~RadEyeSensor(){}

bool RadEyeSensor::connected()
{
    // Can be used to check if a sensor is still connected
    return RadEye_.IsPortOpen();
}

void RadEyeSensor::run()
{
   
            std::vector<std::string> results = this->readRadEye();
            if (results.size() > 0)
            {

                bool binary_num[8]; 
                this->decToBinary(strtol(results[4].c_str(), 0, 16),binary_num); 
                
                for(int i = 0;i < this->radiation_type_.size();i++)
                {
                    radeye_msgs::Radeye msg;
                    msg.units = std::stoi(results[(2*i)+1]);
                    if ((msg.units == 2) | (msg.units == 10))
                    {
                        msg.measurement = stoi(results[(2*i)]);
                        msg.total_dose = stoi(results[6]);
                    }
                    else
                    {
                        msg.measurement = stoi(results[(2*i)])/100.0;
                        msg.total_dose = stoi(results[6])/100.0;
                    }

                    msg.overloaded = binary_num[1];
                    msg.alarm = binary_num[2];
                    msg.lowbattery = binary_num[5];
                    msg.model = results[5];

                    msg.radiation_type = this->radiation_type_[i];
                    msg.header.frame_id = this->frame_id_;
                    msg.header.stamp = ros::Time::now();
                    rad_pub_[i].publish(msg);
                }

                
            }

        
}

std::vector<std::string> RadEyeSensor::readRadEye()
{
            
            std::string rec = this->RadEye_.ReadSerial();
            //std::cout << "data read: "<< rec << std::endl;
            int start_string = -1;
            int end_string = -1;
            
            // Scan recieved data for start and stop bits
            for(int i = 0;i<rec.size();i++)
            {
                if (rec[i] == 0x02)
                {
                    start_string = i+1 ;
                } 
                else if (rec[i] == 0x03)
                {
                    end_string = i-2 ;
                    
                }
            }

            
            if ((start_string != -1) & (end_string != -1))
            {
                //check all the data was recieve correctly and reformat
                std::istringstream payload(rec.substr(start_string,end_string));
                std::vector<std::string> results(std::istream_iterator<std::string>{payload},
                                 std::istream_iterator<std::string>());     
                return results;
            }   
            else
            {
                //return empty vector if the data is bad
                ROS_WARN("Ill formatted data, check if the sensor is aligned correctly");
		std::cout << rec << std::endl;
                std::vector<std::string>results;
                return results;
            }
}

void RadEyeSensor::decToBinary(int n, bool *binary_num) 
{

    //Convert decimal number to binary 
    //Used to decode the sensor flags

    int i = 0; 
    while (i < 8) 
    { 
        binary_num[i] = (bool)(n % 2); 
        n = n / 2; 
        i++; 
    } 

} 
