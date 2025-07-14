# Package Usage

A package for interfacing with the ThermoFisher RadEye sensors.

Currently implemented are the G10 and SX families of devices, however this node can be easily extened for use with other devices, I just do not have access to others currently. 

Inside the package you will find a udev rule which can be copied into "/etc/udev/rules.d/". This will make the radeye sensor always appear on /dev/radeye to save issues with usb ordering.

## radeye_node & radeye_daisychain_node

The sensor being used will be detected when plugged into the docking station with data being published on a topic starting with /Radeye/radeye*SERIALNUMBER*_*READINGNUMBER*/data, where the serial number relates to the device and the reading number deals with sensors which return multiple values. 

It has several ROS 2 parameters that can be set at runtime (e.g., in a launch file):
* `serial_port`: The port the radeye sensor is plugged into. Default: `/dev/radeye` for single, `/dev/ttyACM1` for daisychain.
* `frame_id`: The TF frame to publish the sensor data in. Default: `RadEye`
* `measurement_units`: Allows changing of the sensor units. Default: `""` -> uSv/h (see below)
* `additional_settings`: A way to pass in any additional settings. Default: `""` (see below)
 

#### Returned Values

For understanding the returned values please refer to the included user guide, specifically the automatic sending section for the appropriate sensors: 
i.e. 
* SX:  5.12.10
* G10: 5.17.9

Here you will find the lookup between the units part of the Radeye message and real world units.

Dose rate and total dose have been adjusted to display in :
μR/h, μrem/h μSv/h, cps, cpm, Bq, dps, dpm, Bq/cm2 units respectively

e.g. a value of 584000 when measuring in μsV/h = 5.84msV/H. However in μR/h this would be 584 mR/h.

#### Setting Measurement Units

For selecting output units please see the measuring units sections for the appropriate sensor:

* SX:  5.12.4.5
* G10: 5.17.3.5

These values can be set using the measurement_units argument, requiring the user to calculate the hex value relating to the bits which need to be set in hex and appended to the characters uW. e.g. 
for the G10 to output the measured value in sv/h with the total dose rate in sv/h in scaler mode the bit numbers would be:

* bit number 7654 3210
* bits set   0010 0101
* hex format    2    5
* to send         uW25

Unless you have a specific need I would always use the sensor in scaler mode and with the total dose units matching the instantanious measurement units.

It should be noted that not all unit types are compatible when setting current measurement units and total dose. When cps,cpm,bq,dps or dpm are used the total dose coming off the sensor is not used (does not increase).

#### Additional Settings

For each sensor there are many other settings which can be altered, which can be found within the user guide. These settings can be set in the sensor using the additional_settings parameter. The values to send can be calculated in the same way as the measurement units using the table of flags to calculate the required hex value. These hex values should then be appended to the characters show at the top of each configuration flag table (kW,jW,KW,fW). ,If you require to change multiple settings they can sent using comma seperation e.g. jW0000,fW24,JW2A

## radeye_radiation_point_visualiser.py

This node subscribes to `Radeye` messages and publishes a `PointCloud2` message containing the radiation measurements positioned in the world using TF.

ROS 2 Parameters:
* `pointcloud_name`: Base name for the output PointCloud2 topic. Default: `radeye_measurements`.
* `z_height`: Can be used to override the z-coordinate of the sensor's TF frame. Default: `None`.
* `sensor_frame`: The target TF frame for the point cloud. Default: `radeye`.
* `config_file`: Path to the JSON config file for sensors, relative to the package share directory. The config file should contain an array of "Topics", with each element containing:
  * `SubscriberName`: The topic name to subscribe to.
  * `RadiationValue`: The radiation value (0:Unknown, 1:alpha, 2:beta, 4:gamma, 8:neutron).
  * `RadiationType`: A string label for the radiation type (e.g., "alpha", "beta", "gamma").

See config/example_topics.json for an example.

services:
* GenCSV: call with data set to true to output the current radiation point cloud to a csv file
* ClearRadiationCloud clears with data set to true will clear the current radiation point cloud

## random_tf.py

Also included is a TF publisher used for testing the radiation point cloud when no robot is being used.
This will publish a wandering tf in frame RadEye
## Message Structure

* header   # standard ros header
* overloaded # flag goes high when the sensor is overloaded
* alarm  # flag goes high when the alarm value for radiation is excceded 
* lowbattery # flag indicating low battery
* model # the sensor model being used 
* total_dose # total accumulated dose 
* units # units being used (see below)
* radiation_type # radiation type being detected by this sensor (if known)
* measurement # the instantanious measurement (however there seems to be a 10 second integration window on this value)

The returned units have the following meanings:
* 0 Display unit Sv/h
* 1 Display unit Gy/h
* 2 Display unit R/h
* 3 Display unit cpm
* 5 Display unit cps
* 6 Display unit Bq
* 7 Display unit dpm 
* 8 Display unit Bq/cm2
* 9 Display unit dps
* 10 Display unit rem/h

## ToDo

Look into if the nuclide table is of any use on the go (although this will be hard to test without real rad sources)
