#!/usr/bin/env python

import json
import time
import csv
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import tf2_ros
from rclpy.duration import Duration
import numpy as np 
import sensor_msgs.msg as smsg 
from sensor_msgs import point_cloud2 as pc2
from radeye_msgs.msg import Radeye
from radeye_msgs.srv import GenCSV, ClearRadiationData

"""
This node stores Radeye sensor messages and publishes a pointcloud with their locations (x,y,z), and radiation values.
"""

class RadCloud(Node):
	"""A class to store Radeye sensor messages and publish a pointcloud."""
	def __init__(self):
		super().__init__("radiation_to_pcl")

		# Load params
		self.declare_parameter("pointcloud_name", "radeye_measurements")
		self.declare_parameter("config_file", "config/magnox_radeye_topics.json")
		self.declare_parameter("sensor_frame", "radeye")
		self.declare_parameter("z_height", -1.0) # Use -1.0 as default for no override

		pointcloud_topic = self.get_parameter("pointcloud_name").get_parameter_value().string_value
		config_file_path = self.get_parameter("config_file").get_parameter_value().string_value
		self._sensor_frame = self.get_parameter("sensor_frame").get_parameter_value().string_value
		z_height_param = self.get_parameter("z_height").get_parameter_value().double_value

		if z_height_param >= 0.0:
			self._z_height = z_height_param
			self._z_flag = True
		else:
			self._z_height = None
			self._z_flag = False

		package_share_directory = get_package_share_directory('radeye')
		self._data_file = package_share_directory + "/datasets/" + str(time.ctime()) +".csv"

		if not config_file_path:  # data type is unknown if undefined
			self._field_names = ["radiation"]
			self._radiation_topics = ["radiationTopic"]
			self._radiation_values = ["0"]
		else:
			self._field_names = []
			self._radiation_topics = []
			self._radiation_values = []
			config_file = package_share_directory + "/" + config_file_path
			with open(config_file) as json_file:
				json_data = json.load(json_file)

			for i in json_data["Topics"]:
				self._field_names.append(i["RadiationType"])
				self._radiation_topics.append(i["SubscriberName"])
				self._radiation_values.append(i["RadiationValue"])

		self._sensor_subscribers = []
		
		for i in self._radiation_topics:
			self._sensor_subscribers.append(self.create_subscription(Radeye, i, self.callback, 10))
			self.get_logger().info("subscribed to: " + i)

		self.genCSV_service = self.create_service(GenCSV, 'gen_csv', self.csv_service)
		self.clearData_service = self.create_service(ClearRadiationData, 'clear_radiation_data', self.clear_service)
		
		#generate a point cloud for all of the sensors being used plus one for the combined data
		self._publishers =[]
		for i in self._field_names:
			self._publishers.append(rospy.Publisher(pointcloud_topic+"_"+str(i), smsg.PointCloud2, queue_size=2))
		self._seq = 0
		self._publishers.append(rospy.Publisher(pointcloud_topic, smsg.PointCloud2, queue_size=2))

		#TF buffer to handle transforming location where data was taken, to a global frame, e.g. map
		self._tf_buffer = tf2_ros.Buffer()
		self._listener = tf2_ros.TransformListener(self._tf_buffer)

		#Buffer where all inbound radiation data will be held
		self._data_buffer = {}

		for i in self._radiation_values:
			self._data_buffer[i] = [] 
		self._pc = smsg.PointCloud2()

		self.timer = self.create_timer(1.0, self.publish_pcl)

	def callback(self, msg):
		try:
			#Find position of the radiation sensor (based on the frame in the message header) in the global frame, e.g. map
			self.get_logger().debug("success 0")
			frame_trans = self._tf_buffer.lookup_transform(self._sensor_frame, msg.header.frame_id, msg.header.stamp, timeout=Duration(seconds=1.0))
			self.get_logger().debug("success 1")
			newData = np.asarray([[  frame_trans.transform.translation.x, frame_trans.transform.translation.y, frame_trans.transform.translation.z, self.rosmsg_to_data(msg.measurement)]], dtype=np.float32)
			self.get_logger().debug("success 2")

			radiation_type_str = str(msg.radiation_type)
			if len(self._data_buffer[radiation_type_str]) == 0:
				self._data_buffer[radiation_type_str] = newData
				self.get_logger().debug("success 3")
			else:
				self.get_logger().debug("success 4")
				self._data_buffer[radiation_type_str] = np.concatenate((self._data_buffer[radiation_type_str], newData), axis=0) #Add new data to the existing buffer

			self.get_logger().info("Added data to buffer")

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			self.get_logger().error("TF lookup failed")
		return


	def csv_service(self, request, response):
		try:
			if request.data == True:
				fnames = ["x","y","z"]
				for key in self._field_names:
					fnames.append(key)
				
				self.get_logger().info(f"CSV field names: {fnames}")
				filename = self._data_file
				with open(filename, 'w') as csvfile:
					writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
					writer.writerow(fnames)
					for p in pc2.read_points(self._pc, field_names = fnames, skip_nans=False):
						self.get_logger().debug(f"Writing point to CSV: {p}")
						writer.writerow(p)		
			response.success = True
		except Exception as e:
			self.get_logger().error(f"Failed to generate CSV: {e}")
			response.success = False
		return response

	def clear_service(self, request, response):
		try:
			if request.data == True:
				self._data_buffer = {}

				for i in self._radiation_values:
					self._data_buffer[i] = [] 
			return ClearRadiationDataResponse(True)
		except:
			return ClearRadiationDataResponse(False)

	def rosmsg_to_data(self, msg):

		if not isinstance(msg, float):
			try:
				msg = float(msg)
			except (ValueError, TypeError):
				msg = float('nan')
				self.get_logger().warn("Could not convert data to float")

		return msg
	
	def publish_pcl(self):
		
		msg_data = self._data_buffer.copy()
		total_points = 0 
		count = 0
		for key in msg_data:
			
			total_points += len(msg_data[key])

			if len(msg_data[key]) != 0:
				unused = self.populate_pc(len(msg_data[key]),count,msg_data,[key])  
			count +=1	

			self._pc = self.populate_pc(total_points,count,msg_data,msg_data.keys())

	def populate_pc(self, no_of_points, count, msg_data, keys):
		pc = smsg.PointCloud2()
		pc.header.frame_id = self._sensor_frame
		pc.height = 1 #With height=1, data can be unordered             
		pc.width = no_of_points

		pc.fields = [
			smsg.PointField("x", 0, smsg.PointField.FLOAT32, 1),
			smsg.PointField("y", 4, smsg.PointField.FLOAT32, 1),
			smsg.PointField("z", 8, smsg.PointField.FLOAT32, 1)
		]
		for i in range(0,len(self._field_names)):
			pc.fields.append(smsg.PointField(str(self._field_names[i]), 12+(4*i), smsg.PointField.FLOAT32, 1))

		pc.is_bigendian = False
		pc.point_step = len(pc.fields) * 4 # 4 bytes per field
		pc.row_step = pc.point_step * no_of_points #Size of a row step, for height=1 this is the length of all the data
		pc.is_dense = True


		msg_data_reformated = []
		for key in keys:
			for i in range(0,len(self._radiation_values)):
				if int(key) == int(self._radiation_values[i]):
					rad_position = i+3
					break

			for i in msg_data[key]:
				temp = [float('nan')]*len(pc.fields)
				temp[0:3] = i[0:3]
				temp[rad_position] = i[3]

				if self._z_flag:
					temp[2] = self._z_height

				msg_data_reformated += temp

		if msg_data_reformated:
			pc.data = np.asarray(msg_data_reformated, dtype=np.float32).tobytes()
		else:
			pc.data = b''

		pc.header.stamp = self.get_clock().now().to_msg()
		pc.header.seq = self._seq
		self._publishers[count].publish(pc)

		return pc

def main(args=None):
	rclpy.init(args=args)
	print("Starting radiation data to pointcloud node")
	print("radiation data can be exported to csv using service 'gen_csv' and the cloud can be reset using 'clear_radiation_data'")
	rad_cloud_node = RadCloud()
	rclpy.spin(rad_cloud_node)
	rad_cloud_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
