#!/usr/bin/env python
import select
import socket
import struct
import time
import numpy as np
# import transforms3d
from ImuPacket import ImuPacket

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

imu = ImuPacket()


BNO055_RX_PORT = 27114
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sock.bind(("0.0.0.0", BNO055_RX_PORT))


def talker():


	pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
	rospy.init_node('moab_imu_node', anonymous=True)
	rate = rospy.Rate(100) # 10hz

	_imu_msg = Imu()

	while not rospy.is_shutdown():

		try:
			pkt, addr = sock.recvfrom(128)
			imu.parse(pkt)
		except socket.error:
			pass

		else:
			## ROS doesn't have transforms3d module
			# rot = transforms3d.pub.publish(_imu)quaternions.quat2mat([imu.qw, imu.qx, imu.qy, imu.qz])
			# roll, pitch, _yaw = transforms3d.euler.mat2euler(rot)
			# print(imu.qw, imu.qx,imu.qy,imu.qz)

			qw = imu.qw/16384.0
			qx = imu.qx/16384.0
			qy = imu.qy/16384.0
			qz = imu.qz/16384.0

			r11 = 1.0 - 2.0*qy**2 - 2.0*qz**2
			r12 = 2.0*qx*qy - 2.0*qz*qw
			r13 = 2.0*qx*qz + 2.0*qy*qw
			r21 = 2.0*qx*qy + 2.0*qz*qw
			r22 = 1.0 - 2.0*qx**2 - 2.0*qz**2
			r23 = 2.0*qy*qz - 2.0*qx*qw
			r31 = 2.0*qx*qz - 2.0*qy*qw
			r32 = 2.0*qy*qz + 2.0*qx*qw
			r33 = 1.0 - 2.0*qx**2.0 - 2.0*qy**2

			rot2 = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

			yaw = np.arctan2(r21,r11)
			pitch = np.arcsin(-r31)
			roll = np.arctan2(r32,r33)

			# print(rot2)

			# print("roll %.2f pitch %.2f yaw %.2f" %(roll,pitch,yaw))
			print("qw %.2f qx %.2f qy %.2f qz %.2f" %(qw,qx,qy,qz))
			
			_imu_msg.header.stamp = rospy.Time.now()
			_imu_msg.header.frame_id = 'imu_frame'
			_imu_msg.orientation.x = qx #roll
			_imu_msg.orientation.y = qy #pitch
			_imu_msg.orientation.z = qz #yaw
			_imu_msg.orientation.w = qw 

			# _imu_msg.orientation.x = roll
			# _imu_msg.orientation.y = pitch
			# _imu_msg.orientation.z = yaw


		# hello_str = "hello world %s" % rospy.get_time()
		# rospy.loginfo(hello_str)

			pub.publish(_imu_msg)

		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass