#ifndef RADEYE_H
#define RADEYE_H

#include "rclcpp/rclcpp.hpp"
#include "radeye/RadEyeCom.h"
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <radeye_msgs/msg/radeye.hpp>

using namespace std::chrono_literals;

class RadEyeSensorNode : public rclcpp::Node
{

    //Class to interface with the thermofisher radeye sensors. 
    //Currently only implemented the G-10 and SX sensors 

    public:
        RadEyeSensorNode();
        ~RadEyeSensorNode();
        std::vector<std::string>readRadEye();
        void decToBinary(int, bool*);
        bool connected();

    private:
        void run();

        std::string serial_port_;
        std::string frame_id_;
        std::string unit_name_;
        std::vector<rclcpp::Publisher<radeye_msgs::msg::Radeye>::SharedPtr> rad_pub_;
        std::vector<int> radiation_type_;
        RadEyeCom RadEye_;
        rclcpp::TimerBase::SharedPtr timer_;
};

RadEyeSensorNode::RadEyeSensorNode() : Node("radeye_node")
{
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<std::string>("frame_id", "RadEye");
    this->declare_parameter<std::string>("measurement_units", "");
    this->declare_parameter<std::string>("additional_settings", "");

    this->get_parameter("serial_port", this->serial_port_);
    this->get_parameter("frame_id", this->frame_id_);
    std::string measurement_units, additional_settings;
    this->get_parameter("measurement_units", measurement_units);
    this->get_parameter("additional_settings", additional_settings);

    RCLCPP_INFO(this->get_logger(), "Connecting to: %s", this->serial_port_.c_str());
    this->RadEye_.init(this->serial_port_);
    if (this->connected() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No sensor cable detected!");
        rclcpp::shutdown();
        return;
    }
    this->RadEye_.ActivateCyclicSending();
    this->RadEye_.ClearAccumulatedDose();
    this->unit_name_ = "-1"; // -1 is returned when no sensor is present
    int count = 0;
    
    while (this->unit_name_== "-1" && rclcpp::ok())
    {
        this->unit_name_ = std::to_string(this->RadEye_.ReadDeviceSerialNumber());
        rclcpp::sleep_for(500ms);
        RCLCPP_INFO(this->get_logger(), "Read unit serial: %s", this->unit_name_.c_str());
        count++;
        if (count == 20)
        {
            RCLCPP_ERROR(this->get_logger(), "No sensor detected!");
            rclcpp::shutdown();
            return;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Detected unit no: %s", this->unit_name_.c_str());
    std::vector<std::string> model;
    count = 0;
    while (model.size() == 0 && rclcpp::ok())
    {
        model = this->readRadEye();
        rclcpp::sleep_for(500ms);
        count++;    
        if (count == 20)
        {
            RCLCPP_ERROR(this->get_logger(), "No model detected!");
            rclcpp::shutdown();
            return;
        }    
    }
    RCLCPP_INFO(this->get_logger(), "Detected model: %s", model[5].c_str());
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
            RCLCPP_INFO(this->get_logger(), "Changing Measurement Units");
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
            RCLCPP_WARN(this->get_logger(), "Changing Measurement Units");
        }
    }
    else
    {
        // Add extra sensors here
        RCLCPP_ERROR(this->get_logger(), "Sensor model not implemented! (feel free to add)");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "additional settings %s", additional_settings.c_str());
    std::string setting = "";
    for(std::size_t i = 0; i <= additional_settings.size();i++) 
    {
        if (i == additional_settings.size())
        {
            RCLCPP_INFO(this->get_logger(), "setting %s", setting.c_str());
            this->RadEye_.CommandString(setting);
            setting = "";
        }
        else if (additional_settings[i] == ',')
        {
            RCLCPP_INFO(this->get_logger(), "setting %s", setting.c_str());
            this->RadEye_.CommandString(setting);
            setting = "";
        } else
        {
            setting+= additional_settings[i] ; 
        }
    }

    for(std::size_t i = 0; i < this->radiation_type_.size();i++)
    {
        std::string topic_name = "RadEye" + this->unit_name_ + "_" + std::to_string(i) + "/data";
        this->rad_pub_.push_back(this->create_publisher<radeye_msgs::msg::Radeye>(topic_name, 10));
        RCLCPP_INFO(this->get_logger(), "Publishing on %s", topic_name.c_str());
    }

    timer_ = this->create_wall_timer(1s, std::bind(&RadEyeSensorNode::run, this));
}

RadEyeSensorNode::~RadEyeSensorNode(){}

bool RadEyeSensorNode::connected()
{
    // Can be used to check if a sensor is still connected
    return RadEye_.IsPortOpen();
}

void RadEyeSensorNode::run()
{
   
            std::vector<std::string> results = this->readRadEye();
            if (results.size() > 0)
            {

                bool binary_num[8]; 
                this->decToBinary(strtol(results[4].c_str(), 0, 16),binary_num); 
                
                for(std::size_t i = 0;i < this->radiation_type_.size();i++)
                {
                    auto msg = radeye_msgs::msg::Radeye();
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
                    msg.header.stamp = this->get_clock()->now();
                    rad_pub_[i]->publish(msg);
                }
            }
}

std::vector<std::string> RadEyeSensorNode::readRadEye()
{
            
            std::string rec = this->RadEye_.ReadSerial();
            //std::cout << "data read: "<< rec << std::endl;
            int start_string = -1;
            int end_string = -1;
            
            // Scan recieved data for start and stop bits
            for(std::size_t i = 0;i<rec.size();i++)
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
                RCLCPP_WARN(this->get_logger(), "Ill formatted data, check if the sensor is aligned correctly");
		        RCLCPP_DEBUG(this->get_logger(), "Received: %s", rec.c_str());
                std::vector<std::string>results;
                return results;
            }
}

void RadEyeSensorNode::decToBinary(int n, bool *binary_num)
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

#endif // RADEYE_H
