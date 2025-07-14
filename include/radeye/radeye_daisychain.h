#ifndef RADEYE_DAISYCHAIN_H
#define RADEYE_DAISYCHAIN_H

#include "rclcpp/rclcpp.hpp"
#include "radeye/RadEyeCom.h"
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <chrono>
#include <radeye_msgs/msg/radeye.hpp>
#include <algorithm>

using namespace std::chrono_literals;

/*
THIS NEEDS REWRITTING USING OBJECTS RATHER THAN VECTORS ON VECTORs ON VECTORS
*/

class RadEyeDaisychainNode : public rclcpp::Node
{

    //Class to interface with the thermofisher radeye sensors.
    //Currently only implemented the G-10 and SX sensors

public:
    RadEyeDaisychainNode();
    ~RadEyeDaisychainNode();
    std::vector<std::vector<std::string>> readRadEye();
    void decToBinary(int, bool *);
    bool connected();

private:
    void run();
    std::string serial_port_;
    std::string frame_id_;
    int expected_devices_;
    std::vector<std::string> unit_names_;
    std::vector<std::tuple<std::string, int, rclcpp::Publisher<radeye_msgs::msg::Radeye>::SharedPtr>> rad_pub_;
    std::vector<std::pair<std::string, std::vector<int>>> radiation_type_;
    RadEyeCom RadEye_;
    rclcpp::TimerBase::SharedPtr timer_;
};

RadEyeDaisychainNode::RadEyeDaisychainNode() : Node("radeye_daisychain_node")
{
    this->declare_parameter<int>("expected_devices", 2);
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM1");
    this->declare_parameter<std::string>("frame_id", "RadEye");
    this->declare_parameter<std::string>("measurement_units", "");
    this->declare_parameter<std::string>("additional_settings", "");

    this->get_parameter("expected_devices", this->expected_devices_);
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
    std::string unit_name_ = "-1"; // -1 is returned when no sensor is present
    int count = 0;
    int device_count = 0;
    RCLCPP_INFO(this->get_logger(), "Expecting %d Sensors", this->expected_devices_);

    while (device_count != this->expected_devices_ && rclcpp::ok())
    {
        //unit_name_ = std::to_string(this->RadEye_.ReadDeviceSerialNumber());
        std::vector<std::vector<std::string>> radeye_data;
        radeye_data = this->readRadEye();

        for (size_t i = 0; i < radeye_data.size(); i++)
        {
            std::vector<std::string> model;
            model = radeye_data[i];
            if (model.size() > 5)
            {
                unit_name_ = model[5];
                if (std::find(this->unit_names_.begin(), this->unit_names_.end(), unit_name_) != this->unit_names_.end())
                {
                    RCLCPP_DEBUG(this->get_logger(), "%s already exists", unit_name_.c_str());
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "%s found", unit_name_.c_str());
                    this->unit_names_.push_back(unit_name_);
                    device_count++;
                }
            }
        }
        rclcpp::sleep_for(500ms);
        count++;
        if (count == 100)
        {
            RCLCPP_ERROR(this->get_logger(), "Too few sensors detected!");
            rclcpp::shutdown();
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Detected unit nos: ");
    for (size_t i = 0; i < this->unit_names_.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "- %s", this->unit_names_[i].c_str());
    }

    for (size_t i = 0; i < this->unit_names_.size(); i++)
    {

        if (this->unit_names_[i] == "FH41B2")
        {
            std::vector<int> r{4};
            this->radiation_type_.push_back(std::pair<std::string, std::vector<int>>("FH41B2", r)); //see Radeye message file for type definitions
        }
        else if (this->unit_names_[i] == "SX")
        {
            std::vector<int> r{1, 2};
            this->radiation_type_.push_back(std::pair<std::string, std::vector<int>>("SX", r));
        }
        else
        {
            // Add extra sensors here
            RCLCPP_ERROR(this->get_logger(), " Sensor model not implemented! (feel free to add)");
            rclcpp::shutdown();
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "additional settings: %s", additional_settings.c_str());
    std::string setting = "";
    for (size_t i = 0; i <= additional_settings.size(); i++)
    {
        if (i == additional_settings.size())
        {
            this->RadEye_.CommandString(setting);
            setting = "";
        }
        else if (additional_settings[i] == ',')
        {
            this->RadEye_.CommandString(setting);
            setting = "";
        }
        else
        {
            setting += additional_settings[i]; 
        }
    }

    for (size_t i = 0; i < this->radiation_type_.size(); i++)
    {
        for (size_t j = 0; j < this->radiation_type_[i].second.size(); j++)
        {
            std::string topic_name = "RadEye" + this->radiation_type_[i].first + "_" + std::to_string(j) + "/data";
            auto pub = this->create_publisher<radeye_msgs::msg::Radeye>(topic_name, 10);
            this->rad_pub_.push_back(std::make_tuple(this->radiation_type_[i].first, j, pub));
            RCLCPP_INFO(this->get_logger(), "Publishing on %s", topic_name.c_str());
        }
    }

    timer_ = this->create_wall_timer(1s, std::bind(&RadEyeDaisychainNode::run, this));
}

RadEyeDaisychainNode::~RadEyeDaisychainNode() {}

bool RadEyeDaisychainNode::connected()
{
    // Can be used to check if a sensor is still connected
    return RadEye_.IsPortOpen();
}

void RadEyeDaisychainNode::run()
{

    std::vector<std::vector<std::string>> rad_data = this->readRadEye();
    RCLCPP_DEBUG(this->get_logger(), "rad_data length: %zu", rad_data.size());
    for (size_t j = 0; j < rad_data.size(); j++)
    {
        std::vector<std::string> data = rad_data[j];
        if (data.size() > 0)
        {
            try
            {
                bool binary_num[8];
                this->decToBinary(strtol(data[4].c_str(), 0, 16), binary_num);

                size_t publisher_loops = 1;
                int rad_type = 0;

                for (size_t i = 0; i < this->radiation_type_.size(); i++)
                {
                    if (data[5] == this->radiation_type_[i].first)
                    {
                        publisher_loops = this->radiation_type_[i].second.size();
                        rad_type = i; 
                    }
                }

                RCLCPP_DEBUG(this->get_logger(), "publisher loops: %zu", publisher_loops);
                for (size_t i = 0; i < publisher_loops; i++)
                {
                    auto msg = radeye_msgs::msg::Radeye();
                    msg.units = std::stoi(data[(2 * i) + 1]);
                    if ((msg.units == 2) | (msg.units == 10))
                    {
                        msg.measurement = std::stoi(data[(2 * i)]);
                        msg.total_dose = std::stoi(data[6]);
                    }
                    else
                    {
                        msg.measurement = std::stoi(data[(2 * i)]) / 100.0;
                        msg.total_dose = std::stoi(data[6]) / 100.0;
                    }

                    msg.overloaded = binary_num[1];
                    msg.alarm = binary_num[2];
                    msg.lowbattery = binary_num[5];
                    msg.model = data[5];

                    msg.radiation_type = this->radiation_type_[rad_type].second[i];
                    msg.header.frame_id = this->frame_id_;
                    msg.header.stamp = this->get_clock()->now();

                    for (size_t k = 0; k < this->rad_pub_.size(); k++)
                    {
                        RCLCPP_DEBUG(this->get_logger(), "%s, %d   %s, %zu", std::get<0>(rad_pub_[k]).c_str(), std::get<1>(rad_pub_[k]), msg.model.c_str(), i);
                        if ((std::get<0>(rad_pub_[k]) == msg.model) & (std::get<1>(rad_pub_[k]) == i))
                         {
                            std::get<2>(rad_pub_[k])->publish(msg);
                         }
                    }
                }
            }
            catch (std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Ill-formatted data: %s", e.what());
            }
        }
    }
}

std::vector<std::vector<std::string>> RadEyeDaisychainNode::readRadEye()
{

    std::string rec = this->RadEye_.ReadSerial();
    RCLCPP_DEBUG(this->get_logger(), "data read: %s", rec.c_str());
    int start_string = -1;
    int end_string = -1;
    std::vector<std::vector<std::string>> rad_data;

    // Scan recieved data for start and stop bits
    for(std::size_t i = 0; i < rec.size(); i++)
    {
        if (rec[i] == 0x02)
        {
            start_string = i + 1;
        }
        else if (rec[i] == 0x03)
        {
            end_string = i - 2;
        }

        if ((start_string != -1) & (end_string != -1))
        {
            //check all the data was recieve correctly and reformat
            std::istringstream payload(rec.substr(start_string, end_string));
            std::vector<std::string> results(std::istream_iterator<std::string>{payload},
                                             std::istream_iterator<std::string>());
            rad_data.push_back(results);
            start_string = -1;
            end_string = -1;
        }
    }

    if (rad_data.size() == 0)
    {
        //return empty vector if the data is bad
        RCLCPP_WARN(this->get_logger(), "Ill formatted data, check if the sensor is aligned correctly");
    }
    return rad_data;
}

void RadEyeDaisychainNode::decToBinary(int n, bool *binary_num)
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

#endif // RADEYE_DAISYCHAIN_H
