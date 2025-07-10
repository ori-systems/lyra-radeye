#include "ros/ros.h"
#include "radeye/RadEyeCom.h"
#include <iostream>
#include <string.h>
#include <radeye_msgs/Radeye.h>

#include <algorithm>


/*
THIS NEEDS REWRITTING USING OBJECTS RATHER THAN VECTORS ON VECTORs ON VECTORS
*/

class RadEyeSensor
{

    //Class to interface with the thermofisher radeye sensors.
    //Currently only implemented the G-10 and SX sensors

public:
    RadEyeSensor(ros::NodeHandle);
    ~RadEyeSensor();
    std::vector<std::vector<std::string>> readRadEye();
    void decToBinary(int, bool *);
    void run();
    bool connected();

private:
    std::string serial_port_;
    std::string frame_id_;
    int expected_devices_;
    std::vector<std::string> unit_names_;
    ros::NodeHandle n_;
    std::vector<std::tuple<std::string, int, ros::Publisher>> rad_pub_;
    std::vector<std::pair<std::string, std::vector<int>>> radiation_type_;
    RadEyeCom RadEye_;
};

RadEyeSensor::RadEyeSensor(ros::NodeHandle n)
{

    std::string measurement_units, additional_settings;
    this->n_ = n;
    this->n_.param<int>("expected_devices", this->expected_devices_, 2);
    this->n_.param<std::string>("serial_port", this->serial_port_, "/dev/ttyACM1");
    this->n_.param<std::string>("frame_id", this->frame_id_, "RadEye");
    this->n_.param<std::string>("measurement_units", measurement_units, "");
    this->n_.param<std::string>("additional_settings", additional_settings, "");
    std::cout << "Connecting to: " << this->serial_port_ << std::endl;
    this->RadEye_.init(this->serial_port_);
    if (this->connected() == 0)
    {
        ROS_ERROR("No sensor cable detected!");
        exit(0);
    }
    this->RadEye_.ActivateCyclicSending();
    this->RadEye_.ClearAccumulatedDose();
    std::string unit_name_ = "-1"; // -1 is returned when no sensor is present
    int count = 0;
    int device_count = 0;
    std::cout << "Expecting " << this->expected_devices_ << " Sensors" << std::endl;

    while (device_count != this->expected_devices_)
    {
        //unit_name_ = std::to_string(this->RadEye_.ReadDeviceSerialNumber());
        std::vector<std::vector<std::string>> radeye_data;
        radeye_data = this->readRadEye();

        for (int i = 0; i < radeye_data.size(); i++)
        {
            std::vector<string> model;
            model = radeye_data[i];
            if (model.size() > 5)
            {
                unit_name_ = model[5];
                if (std::find(this->unit_names_.begin(), this->unit_names_.end(), unit_name_) != this->unit_names_.end())
                {
                    std::cout << unit_name_ << " already exists" << std::endl;
                }
                else
                {
                    std::cout << unit_name_ << " found" << std::endl;
                    this->unit_names_.push_back(unit_name_);
                    device_count++;
                }
            }
        }
        usleep(500000);
        count++;
        if (count == 100)
        {
            ROS_ERROR("Too few sensors detected!");
            exit(0);
        }
    }

    std::cout << "Detected unit nos: " << std::endl;
    for (int i = 0; i < this->unit_names_.size(); i++)
    {
        std::cout << this->unit_names_[i] << std::endl;
    }

    for (int i = 0; i < this->unit_names_.size(); i++)
    {

        if (this->unit_names_[i] == "FH41B2")
        {
            std::vector<int> r{4};
            this->radiation_type_.push_back(std::pair<std::string, std::vector<int>>("FH41B2", r)); //see Radeye message file for type definitions
            /*if (measurement_units == "")
            {
                this->RadEye_.CommandString("uW25");
            }
            else 
            {
                this->RadEye_.CommandString(measurement_units);
                std::cout <<"Changing Measurement Units" << std::endl;
            }*/
        }
        else if (this->unit_names_[i] == "SX")
        {
            std::vector<int> r{1, 2};
            this->radiation_type_.push_back(std::pair<std::string, std::vector<int>>("SX", r));
            //this->radiation_type_.push_back(std::pair<std::string, int>("SX",1));
            /*if (measurement_units == "")
            {
                this->RadEye_.CommandString("uW40");

            }
            else 
            {
                this->RadEye_.CommandString(measurement_units);
                ROS_WARN("Changing Measurement Units");
            }*/
        }
        else
        {
            // Add extra sensors here
            ROS_ERROR(" Sensor model not implemented! (feel free to add)");
            exit(0);
        }
    }

    std::cout << "additional settings " << additional_settings << std::endl;
    std::string setting = "";
    for (int i = 0; i <= additional_settings.size(); i++)
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

    for (int i = 0; i < this->radiation_type_.size(); i++)
    {
        for (int j = 0; j < this->radiation_type_[i].second.size(); j++)
        {
            this->rad_pub_.push_back(std::tuple<std::string,int, ros::Publisher>(this->radiation_type_[i].first,j,n.advertise<radeye_msgs::Radeye>("RadEye" + this->radiation_type_[i].first + "_" + std::to_string(j) + "/data", 10)));
            std::cout << "Publishing on "
                      << "RadEye" + this->radiation_type_[i].first + "_" + std::to_string(j) + "/data" << std::endl;
        }
    }
}

RadEyeSensor::~RadEyeSensor() {}

bool RadEyeSensor::connected()
{
    // Can be used to check if a sensor is still connected
    return RadEye_.IsPortOpen();
}

void RadEyeSensor::run()
{

    std::vector<std::vector<std::string>> rad_data = this->readRadEye();
    std::cout << "length " << rad_data.size() << std::endl;
    for (int j = 0; j < rad_data.size(); j++)
    {
        std::vector<std::string> data = rad_data[j];
        if (data.size() > 0)
        {

            try
            {
                bool binary_num[8];
                this->decToBinary(strtol(data[4].c_str(), 0, 16), binary_num);

                int publisher_loops = 1;
                int rad_type = 0;

                for (int i = 0; i < this->radiation_type_.size(); i++)
                {
                    if (data[5] == this->radiation_type_[i].first)
                    {
                        publisher_loops = this->radiation_type_[i].second.size();
                        rad_type = i;
                    }
                }

                std::cout << "publisher loops " << publisher_loops << std::endl;
                for (int i = 0; i < publisher_loops; i++)
                {
                    radeye_msgs::Radeye msg;
                    msg.units = std::stoi(data[(2 * i) + 1]);
                    if ((msg.units == 2) | (msg.units == 10))
                    {
                        msg.measurement = stoi(data[(2 * i)]);
                        msg.total_dose = stoi(data[6]);
                    }
                    else
                    {
                        msg.measurement = stoi(data[(2 * i)]) / 100.0;
                        msg.total_dose = stoi(data[6]) / 100.0;
                    }

                    msg.overloaded = binary_num[1];
                    msg.alarm = binary_num[2];
                    msg.lowbattery = binary_num[5];
                    msg.model = data[5];

                    msg.radiation_type = this->radiation_type_[rad_type].second[i];
                    msg.header.frame_id = this->frame_id_;
                    msg.header.stamp = ros::Time::now();

                    for (int k = 0; k < this->rad_pub_.size(); k++)
                    {
                        std::cout << std::get<0>(rad_pub_[k]) << "," << std::get<1>(rad_pub_[k]) << "   "<< msg.model << "," << i<< std::endl;
                        if ((std::get<0>(rad_pub_[k]) == msg.model) & (std::get<1>(rad_pub_[k]) == i))
                         {
                            std::get<2>(rad_pub_[k]).publish(msg);
                         }
                        
                    }
                }
            }
            catch (std::exception &e)
            {
                std::cout << "ill formated data" << std::endl;
            }
        }
    }
}

std::vector<std::vector<std::string>> RadEyeSensor::readRadEye()
{

    std::string rec = this->RadEye_.ReadSerial();
    std::cout << "data read: " << rec << std::endl;
    int start_string = -1;
    int end_string = -1;
    std::vector<std::vector<std::string>> rad_data;

    // Scan recieved data for start and stop bits
    for (int i = 0; i < rec.size(); i++)
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
        ROS_WARN("Ill formatted data, check if the sensor is aligned correctly");
    }
    return rad_data;
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
