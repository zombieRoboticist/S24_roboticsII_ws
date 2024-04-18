import numpy as np
import cv2
import socket
import json
import sys
import threading

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
	import msvcrt
else:
	import termios
	import tty



def getControlEffort(pose):

	kpx=1
	kdx=0
	kix=0
	errorSumx = 0
	errorPreviousx = 0

	angleTol = 0.06
	kpy=1
	kdy=0
	kiy=0
	errorSumy = 0
	errorPreviousy = 0

	kpangle=.03
	kdangle=0
	kiangle=0
	errorSumangle = 0
	errorPreviousangle = 0

	#define ideal pose of the robot
	yOffset=0
	xOffset=.5
	angleOffset=0

	#define unit conversions
	angleScale=100
	distScale=1
	yScale=1

	#get current error
	errorx=xOffset-distScale*pose[0]
	errory=angleOffset - angleScale*pose[2] #yOffset-yScale*pose[1]
	errorAngle=yOffset - yScale*pose[1] #angleOffset-angleScale*pose[2]

	#handle angles
	if abs(pose[2]) <angleTol:
		errory=0

	#update integral error
	errorSumx+=errorx
	errorSumy+=errory
	errorSumangle+=errorAngle

	#determine control effort
	#controlEffort = [errorx*kpx+kix*errorSumx+kdx*(errorx-errorPreviousx),errory*kpy+kiy*errorSumy+kdy*(errory-errorPreviousy),errorAngle*kpangle+kiangle*errorSumangle+kdangle*(errorAngle-errorPreviousangle)]
	controlEffort = [errorx*kpx,errory*kpy,errorAngle*kpangle];

	#update previous error
	errorPreviousx=errorx
	errorPreviousy=errory
	errorPreviousangle=errorAngle

	#return the control effort
	return controlEffort


def estimatePose(frame):
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	corners,ids,rejectedImgPoints = detector.detectMarkers(gray)

	if len(corners) > 0:
		cv2.aruco.drawDetectedMarkers(frame,corners)
		c1 = corners[0][0][0]
		c2 =  corners[0][0][1]
		c3 =  corners[0][0][2]
		c4 =  corners[0][0][3]
	
		v1 = (c2[0] - c1[0], c2[1] - c1[1])
		v2 = (c4[0] - c1[0], c4[1] - c1[1])
		
		v1_ = np.array(v1)
		v2_ = np.array(v1)

		c1_ = np.array(c1)
		c2_ = np.array(c2)
		c3_ = np.array(c3)
		c4_ = np.array(c4)

		coords = ( int ( (c1[0] + c2[0]) // 2) , int( (c1[1] + c4[1]) // 2) ) 
		coords = ( int ( (v1[0] + v2[0]) // 2 + int(c1[0])) , int( (v1[1] + v2[1]) // 2 + int(c1[1])) ) 
		cv2.circle(frame,coords, 5, (0,255,0), 2)

		dist_center = (coords[0] - (frame.shape[0] / 2))
		distance_away = v2[1]/ frame.shape[1]
		h1  = (c1_ - c4_)[1]
		h2 = (c2_ - c3_)[1]
		twist_ratio = (h1 - h2) / ( h1  + h2)
		return (distance_away, dist_center, twist_ratio)
	
	return None
		

def main():
	video = cv2.VideoCapture('/dev/video1')
	rclpy.init()

	node = rclpy.create_node('ros_node')

	# parameters
	stamped = node.declare_parameter('stamped', False).value
	frame_id = node.declare_parameter('frame_id', '').value
	if not stamped and frame_id:
		raise Exception("'frame_id' can only be set when 'stamped' is True")

	if stamped:
		TwistMsg = geometry_msgs.msg.TwistStamped
	else:
		TwistMsg = geometry_msgs.msg.Twist

	pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

	spinner = threading.Thread(target=rclpy.spin, args=(node,))
	spinner.start()

	speed = 0.5
	turn = 1.0
	x = 0.0
	y = 0.0
	z = 0.0
	th = 0.0
	status = 0.0

	twist_msg = TwistMsg()

	if stamped:
		twist = twist_msg.twist
		twist_msg.header.stamp = node.get_clock().now().to_msg()
		twist_msg.header.frame_id = frame_id
	else:
		twist = twist_msg

	try:
		while True:
			ret,frame = video.read()
			pose = estimatePose(frame)
			effort = None


			if pose:
				print(pose)
				#print(getControlEffort(pose)[1], pose[2])
				effort = getControlEffort(pose)

			
			if stamped:
				twist_msg.header.stamp = node.get_clock().now().to_msg()


			if effort:
				twist.linear.x = float(effort[0])
				twist.linear.y = float(effort[1])
				twist.linear.z = 0.0
				twist.angular.x = 0.0
				twist.angular.y = 0.0
				twist.angular.z = float(effort[2])
			else:
				twist.linear.x = 0.0
				twist.linear.y = 0.0
				twist.linear.z = 0.0
				twist.angular.x = 0.0
				twist.angular.y = 0.0
				twist.angular.z = 0.0
			
			pub.publish(twist_msg)

	except Exception as e:
		print(e)

	finally:
		if stamped:
			twist_msg.header.stamp = node.get_clock().now().to_msg()

		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = 0.0
		pub.publish(twist_msg)
		rclpy.shutdown()
		spinner.join()

	
	video.release()
	cv2.destroyAllWindows()

detector = cv2.aruco.ArucoDetector( cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50),cv2.aruco.DetectorParameters())

if __name__ == '__main__':
	main()
