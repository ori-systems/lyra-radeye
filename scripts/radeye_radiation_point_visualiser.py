#!/usr/bin/env python

import sys
import json
import time
import csv
import rospy
import rospkg
import tf2_ros
import numpy as np 
import sensor_msgs.msg as smsg 
from sensor_msgs import point_cloud2 as pc2
from dynamic_reconfigure.server import Server
from std_msgs.msg import Bool
from radeye.msg import Radeye
from radeye.srv import GenCSV,GenCSVResponse,ClearRadiationData,ClearRadiationDataResponse

"""
This node stores Radeye sensor messages and publishes a pointcloud with their locations (x,y,z), and radiation values.
"""

class RadCloud(object):
	"""docstring for Radcloud."""
	def __init__(self):
		rospy.init_node("radiation_to_pcl")


		#load params
		pointcloud_topic = rospy.get_param("~pointcloud_name","radeye_measurements") #Name of the pointcloud to hold radiation data
		config_file = rospy.get_param("~config_file","config/magnox_radeye_topics.json") #file relating the radiation types to radiation types numbers
		self._sensor_frame = rospy.get_param("~sensor_frame","radeye") #Set the frame in which the pointcloud should be published 
		self._z_height = rospy.get_param("~z_height",None) #set x height of point cloud

		if self._z_height == None:
			self._z_flag = False
		else:
			self._z_flag = True

		rospack = rospkg.RosPack()
		self._data_file = rospack.get_path('radeye') + "/datasets/" + str(time.ctime()) +".csv"

		if config_file == None:  # data type is unknown if undefined
			self._field_names = ["radiation"]
			self._radiation_topics = ["radiationTopic"]
			self._radiation_values = ["0"]
		else:
			self._field_names = []
			self._radiation_topics = []
			self._radiation_values = []
			config_file = rospack.get_path('radeye') + "/" + config_file
			with open(config_file) as json_file:
				json_data = json.load(json_file)

			for i in json_data["Topics"]:
				self._field_names.append(i["RadiationType"])
				self._radiation_topics.append(i["SubscriberName"])
				self._radiation_values.append(i["RadiationValue"])



		self._sensor_subscribers = []
		
		for i in self._radiation_topics:
			self._sensor_subscribers.append(rospy.Subscriber(i, Radeye, self.callback))
			rospy.loginfo("subscribed to: "+i)



		self.genCSV = rospy.Service('gen_csv', GenCSV, self.csv_service)
		self.clearData = rospy.Service('clear_radiation_data', ClearRadiationData, self.clear_service)
		
		
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

		rate = rospy.Rate(1)

		while not rospy.is_shutdown():

			self.publish_pcl()
			rate.sleep()

	def callback(self, msg):
		try:
			#Find position of the radiation sensor (based on the frame in the message header) in the global frame, e.g. map
			print "success 0"
			frame_trans = self._tf_buffer.lookup_transform(self._sensor_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(1))
			print "success 1"
			newData = np.asarray([[  frame_trans.transform.translation.x, frame_trans.transform.translation.y, frame_trans.transform.translation.z, self.rosmsg_to_data(msg.measurement)]], dtype=np.float32)
			print "success 2" 

			if len(self._data_buffer[msg.radiation_type]) == 0:
				self._data_buffer[msg.radiation_type] = newData
				print "success 3"
			else:
				print "success 4"
				self._data_buffer[msg.radiation_type] = np.concatenate((self._data_buffer[msg.radiation_type], newData), axis=0) #Add new data to the existing buffer


			print "ADDED DATA TO BUFFER"

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print "ERROR"
		return


	def csv_service(self,req):
		try:
			if req.data == True:
				fnames = ["x","y","z"]
				for key in self._field_names:
					fnames.append(key)
				
				print fnames
				filename = self._data_file
				with open(filename, 'w') as csvfile:
					writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
					writer.writerow(fnames)
					for p in pc2.read_points(self._pc, field_names = fnames, skip_nans=False):
						print p
						writer.writerow(p)		
			return GenCSVResponse(True)
		except:
			return GenCSVResponse(False)

	def clear_service(self,req):
		try:
			if req.data == True:
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
			except:
				msg = float('nan')
				print("Could not convert data to float")

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

	def populate_pc(self,no_of_points,count,msg_data,keys):   #needs cleaning up
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
		pc.point_step =  len(pc.fields)*4 #4 bytes per field, and data.shape[1] gives the number of fields (x, y, z, radiation)
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
				
				if self._z_flag:  #If Z height set to 0.0, rather than sensor height, set third column to 0.0 - original buffer is untouched
					msg_data_reformated[:,2] = self._z_height

				msg_data_reformated += temp


		#print msg_data_reformated		
		msg_data_reformated = np.asarray(msg_data_reformated, dtype=np.float32).tostring()


		pc.data = msg_data_reformated #Byte representations of data

		pc.header.stamp = rospy.Time.now()
		pc.header.seq = self._seq
		self._publishers[count].publish(pc)

		return pc


if __name__ == "__main__":

	try:
		print("Starting radiation data to pointcloud node")
		print("radiation data can be exported to csv using services pub_csv and the cloud can be reset using clear_radiation_data")
		s = RadCloud()
		rospy.spin()
	except rospy.ROSInterruptException: pass
