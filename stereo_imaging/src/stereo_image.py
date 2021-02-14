#! /usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from geometry_msgs.msg import Point
#import sensor_msgs.point_cloud_2 as pc2
import numpy as np
from visualization_msgs.msg import Marker
from tf.transformations import euler_matrix
import math
#import pcl
import time
import struct

bridge_image = CvBridge()

def left_image_raw(image):
	global left_image
	#print("initialised the left_image")
	left_image = bridge_image.imgmsg_to_cv2(image,"bgr8")
	#cv2.namedWindow('image_left_raw', cv2.WINDOW_NORMAL)
	#cv2.imshow('image_left_raw',left_image)
	#cv2.waitKey(50)


def right_image_raw(image):
	global right_image
	
	right_image = bridge_image.imgmsg_to_cv2(image,"bgr8")
	#cv2.namedWindow('image_right_raw', cv2.WINDOW_NORMAL)
	#cv2.imshow('image_right_raw',right_image)
	#cv2.waitKey(50)


def right_camera_properties(right_camera_properties):
	global right_K, right_D, right_R, baseline, right_P

	right_K = np.array(right_camera_properties.K).reshape((3,3))
	#print("Intrinsic right \n",right_K)
	right_D = np.array(right_camera_properties.D)
	right_R = np.array(right_camera_properties.R).reshape((3,3))
	right_P = np.array(right_camera_properties.P).reshape((3,4))
	baseline = right_camera_properties.P[3]/(-right_camera_properties.K[0]) # available only for right camera 
	#print("baseline \n",baseline)

def left_camera_properties(left_camera_properties):
	global left_K, left_D, left_R, left_P, height, width

	left_K = np.array(left_camera_properties.K).reshape((3,3))
	#print("Intrinsic left \n",left_K)
	left_D = np.array(left_camera_properties.D)
	left_R = np.array(left_camera_properties.R).reshape((3,3))
	left_P = np.array(left_camera_properties.P).reshape((3,4))
	height, width = left_camera_properties.height, left_camera_properties.width


def point_cloud_object(point_cloud):
	global cloud

	cloud = point_cloud


def x_y_z_from_pcd(u,v):
	point_step = cloud.point_step
	row_step = cloud.row_step
	array_pos = v*row_step + u*point_step

	bytesX = [ord(x) for x in cloud.data[array_pos:array_pos+4]]
	bytesY = [ord(x) for x in cloud.data[array_pos+4:array_pos+8]]
	bytesZ = [ord(x) for x in cloud.data[array_pos+8:array_pos+12]]

	byte_format=struct.pack('4B', *bytesX)
	X = struct.unpack('f', byte_format)[0]

	byte_format=struct.pack('4B', *bytesY)
	Y = struct.unpack('f', byte_format)[0]

	byte_format=struct.pack('4B', *bytesZ)
	Z = struct.unpack('f', byte_format)[0]

	return (X, Y, Z)

def publish_rotated_pcd(points):

	#transformation from stereo camera to opencv camera
	tf_opencv_camera_stereo_camera = euler_matrix(-1.5707963,0.0,-1.5707963)

	pose = Point()
	stereo_points = list()

	for point in points:
		#projecting points from opencv_camera frame to stereo_camera frame
		#print("Point", point, len(point))
		point = np.reshape(np.array(point), (-1,1))
		#print("reshape point", point)
		point_stereo_camera = np.matmul(tf_opencv_camera_stereo_camera,point)
		pose.x = point_stereo_camera[0]
		pose.y = point_stereo_camera[1]
		pose.z = point_stereo_camera[2]

		stereo_points.append(pose)

	marker_points = Marker()

	marker_points.header.frame_id = "stereo_left_camera"
	marker_points.header.stamp = rospy.Time.now()
	marker_points.header.seq = 1
	
	marker_points.ns = "actual_pcd"
	marker_points.id = 8
	marker_points.type = marker_points.POINTS
	marker_points.action = marker_points.ADD
	
	marker_points.pose.orientation.w = 1.0
	
	marker_points.scale.x = 0.01
	marker_points.scale.y = 0.01
	marker_points.scale.z = 0.01
	
	marker_points.color.r = 1.0
	marker_points.color.g = 1.0
	marker_points.color.b = 0.0
	marker_points.color.a = 1.0

	marker_points.points = stereo_points
	print("All points are transformed and published to visulaize in RVIZ")
	pub_marker.publish(marker_points)
	

def filter(image):
	#red color objects with HSV color space
	hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
	lower_red = np.array([60,120,70])
	upper_red = np.array([255,255,255])
	mask = cv2.inRange(hsv, lower_red, upper_red)

	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	return mask

def detect_objects_mask(left_image):

	gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
	#gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

	left_masked = cv2.adaptiveThreshold(gray_left,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,7,2)

	left_masked = cv2.medianBlur(left_masked,3)

	#left_masked = cv2.erode(left_masked, None, iterations=3)
	#left_masked = cv2.dilate(left_masked, None, iterations=3)

	image, contours,heirarchy = cv2.findContours(left_masked,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	area_centroid = []
	for i in range(len(contours)):
		moments = cv2.moments(contours[i])

		if moments['m00'] > 0:
			cx = cx = int(moments['m10']/moments['m00'])
			cy = int(moments['m01']/moments['m00']) 
			area = moments['m00']
			if area > 5000 and area < 6000:
				#print(contours[i])
				area_centroid.append({"area":area,"cx":cx,"cy":cy,"contour":i})



		
	#print("Area: ")
	#print(len(area_centroid))
	#print(area_centroid)

	for cnt in area_centroid:
		#print("Length of the contours: {}".format(len(contours[cnt.get("contour")])))
		#print(contours[cnt.get("contour")].shape)
		
		contour_points = len(contours[cnt.get("contour")])

		points_xyz=[]
		for point in contours[cnt.get("contour")]:
			u,v = point[0][0], point[0][1]
			#print("U: {}  V: {}".format(u,v))
			x,y,z = x_y_z_from_pcd(u,v)
			if not(math.isnan(x) or math.isnan(y) or math.isnan(z)):
				print("Checked for nan and appending")
				points_xyz.append([x,y,z,1])


		# print(contours[int(contour_points/2)][0][0])
		
		# u,v = contours[0][0][0],contours[int(contour_points/2)][0][0][1]
		# print("U: {}  V: {}".format(u,v))
		# print("XYZ: ",x_y_z_from_pcd(u,v))

		# u,v = contours[int(contour_points/2 + contour_points/4)][0][0],contours[int(contour_points/2 + contour_points/4)][0][1]
		# print("U: {}  V: {}".format(u,v))
		# print("XYZ: ",x_y_z_from_pcd(u,v))

		# u,v = contours[int(contour_points/2 - contour_points/4)][0][0],contours[int(contour_points/2 - contour_points/4)][0][1]
		# print("U: {}  V: {}".format(u,v))
		# print("XYZ: ",x_y_z_from_pcd(u,v))	

		cv2.drawContours(left_image, contours, cnt.get("contour"), (0,255,0), 3)
		u,v = cnt.get("cx"), cnt.get("cy")
		cv2.imshow("gray_left", left_masked)
		cv2.imshow("gray_left_raw", left_image)
		cv2.waitKey(3)
		return points_xyz
	
	

	return list()

def detect_objects(left_image,right_image):

	# left_masked_image = filter(left_image)
	# right_masked_image = filter(right_image)
	gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
	gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
	detector = cv2.SimpleBlobDetector_create()
	l_keypoints = detector.detect(gray_left)
	r_keypoints = detector.detect(gray_right)
	print("size")
	print(len(l_keypoints))
	print(len(r_keypoints))


	# for blob_l,blob_r in l_keypoints,r_keypoints:
	# 	x_l,y_l = blob_l.pt
	# 	x_r,y_r = blob_r.pt
	# 	print("Centroid of the blob left image, X:{} Y:{} ".format(x_l,y_l))
	# 	print("Centroid of the blob left image, X:{} Y:{} ".format(x_r,y_r))
	# 	print("Diameter of the left blob:{} ".format(blob_l.size))
	# 	print("Diameter of the right blob:{} ".format(blob_r.size))

	# 	l_im = cv2.circle(gray_left, (int(x_l),int(y_l)), 5, (255, 255, 0), 2)
	# 	r_im = cv2.circle(gray_right, (int(x_r),int(y_r)), 5, (255, 255, 0), 2)

	# 	l_top_left_x, l_top_left_y = (int(x_l) - int(blob_l.size / 2), int(y_l) -  int(blob_l.size / 2))

	# 	l_bottom_right_x, l_bottom_right_y = (int(x_l) + int(blob_l.size / 2), int(y_l) + int(blob_l.size / 2))

	# 	r_top_left_x, r_top_left_y = (int(x_r) - int(blob_l.size / 2), int(y_r) -  int(blob_r.size / 2))

	# 	r_bottom_right_x, r_bottom_right_y = (int(x_r) + int(blob_r.size / 2), int(y_r) + int(blob_r.size / 2))

	# 	l_im = cv2.rectangle(l_im,(l_top_left_x,l_top_left_y), (l_bottom_right_x,l_bottom_right_y),(0,255,0),5)

	# 	r_im = cv2.rectangle(r_im,(r_top_left_x,r_top_left_y), (r_bottom_right_x,r_bottom_right_y),(0,255,0),5)

	#draw bounding box around the detected blobs
	#top_left_x, top_left_y = (cv2.Round()

	gray_left = cv2.drawKeypoints(gray_left, l_keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	gray_right = cv2.drawKeypoints(gray_right, r_keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)



	left_cylinder,left_cube = l_keypoints[0].pt,l_keypoints[1].pt
	right_cylinder,right_cube = r_keypoints[0].pt,r_keypoints[1].pt

	disparity_cylinder = int(left_cylinder[0])-int(right_cylinder[0])
	disparity_cube = int(left_cube[0])-int(right_cube[0])

	f_pixel = (width * 0.5) / np.tan(1.3962634 * 0.5)
	depth_cylinder = baseline * f_pixel / disparity_cylinder
	depth_cube = baseline * f_pixel / disparity_cube 
	
	print("Disparity Cylinder:{} Cube:{}".format(disparity_cylinder,disparity_cube))
	
	print("Depth Cylinder:{} Cube:{}".format(depth_cylinder,depth_cube))

	X_cylinder = depth_cylinder * int(left_cylinder[0])/f_pixel
	Y_cylinder = depth_cylinder * int(left_cylinder[1])/f_pixel

	print("Cylinder w.r.t camera X:{} Y:{} Z:{}".format(X_cylinder,Y_cylinder,depth_cylinder))

	cv2.imshow("left",gray_left)
	cv2.imshow("right",gray_right)

	cv2.waitKey(3)









def disparity_compute():

	gray_left = cv2.cvtColor(left_rectified, cv2.COLOR_BGR2GRAY)
	gray_right = cv2.cvtColor(right_rectified, cv2.COLOR_BGR2GRAY)
	cv2.imshow("left", gray_left)
	cv2.imshow("right", gray_right)
	stereo = cv2.StereoBM_create(numDisparities=16, blockSize=5) 
	disparity = stereo.compute(gray_left,gray_right)

	# cv.FindStereoCorrespondenceBM(gray_left, gray_right, disparity, state)
	cv2.imshow("disparity", disparity)
	cv2.waitKey(3)


def undistort_rectify_image():
	global left_rectified, right_rectified
	# R = np.eye(3)
	# T = np.array([0.07, 0.0, 0.0])
	# left_calib_R, right_calib_R, left_calib_P, right_calib_P, Q,roi_left, roi_right = cv2.stereoRectify(left_K, left_D, right_K, right_D, left_image.shape[:2], R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.9)

	# print("Calibrated Projection and Rotation")
	# print("Right")
	# print(right_calib_P)
	# print("left")
	# print(left_calib_P)

	leftMapX, leftMapY = cv2.initUndistortRectifyMap(left_K,left_D,left_R,left_P,(width, height), cv2.CV_32FC1)
	left_rectified = cv2.remap(left_image, leftMapX, leftMapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
	rightMapX, rightMapY = cv2.initUndistortRectifyMap(right_K,right_D,right_R,right_P,(width, height), cv2.CV_32FC1)
	right_rectified = cv2.remap(right_image, rightMapX, rightMapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)


#function that initates the subscription of topics
def image_proc():
	global pub_marker

	left_cam = rospy.Subscriber("/stereo/left/camera_info", CameraInfo, callback=left_camera_properties)
	right_cam = rospy.Subscriber("/stereo/right/camera_info", CameraInfo, callback=right_camera_properties)
	left_image = rospy.Subscriber("/stereo/left/image_raw", Image, callback=left_image_raw)
	right_image = rospy.Subscriber("/stereo/right/image_raw", Image, callback=right_image_raw)

	pcd_3d = rospy.Subscriber("/stereo/points2",PointCloud2,point_cloud_object)

	pub_marker = rospy.Publisher("/stereo/actual_pcd",Marker,queue_size=1)

def main():
	
	rospy.init_node("stereo_imaging")
	print("Node Started")
	image_proc()
	time.sleep(5)
	print("Checked for the subscription of topics")
	rospy.wait_for_message("/stereo/points2",PointCloud2)
	print("Point cloud received.....")
	
	while not rospy.is_shutdown():
		object_3d_points = detect_objects_mask(left_image)
		#print("Length of detected points: {}".format(len(object_3d_points)))
		if len(object_3d_points) > 0:
			publish_rotated_pcd(object_3d_points)
		# u,v = detect_objects_mask(left_image)
		# print("Pixel coords: U: {}  V: {}".format(u,v))
		# if (u!=0 and v!=0):
		# 	x,y,z = x_y_z_from_pcd(u,v)
		# 	print(x,y,z)
		# 	print("3d Pose of the object:\n X: {}  Y: {}  Z: {}".format(x,y,z))

		# else:
		# 	print("No objects detected......")

		#cv2.imshow("left_image_raw", left_image)
		#undistort_rectify_image()
		#isparity_compute()
		#cv2.imshow("left_rectified", left_rectified)
		#cv2.imshow("right_rectified", right_rectified)
		#cv2.waitKey(3)
	
	print("Shutting down")
	cv2.destroyAllWindows()

main()