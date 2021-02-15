#! /usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from geometry_msgs.msg import Point,Pose
import numpy as np
from visualization_msgs.msg import Marker
from tf.transformations import euler_matrix
import math
import time
import struct

bridge_image = CvBridge()

def left_image_rect(image):
	global left_image

	left_image = bridge_image.imgmsg_to_cv2(image,"bgr8")


def right_image_raw(image):
	global right_image
	
	right_image = bridge_image.imgmsg_to_cv2(image,"bgr8")



def right_camera_properties(right_camera_properties):
	global right_K, right_D, right_R, baseline, right_P

	right_K = np.array(right_camera_properties.K).reshape((3,3))

	right_D = np.array(right_camera_properties.D)
	right_R = np.array(right_camera_properties.R).reshape((3,3))
	right_P = np.array(right_camera_properties.P).reshape((3,4))
	baseline = right_camera_properties.P[3]/(-right_camera_properties.K[0]) # available only for right camera 


def left_camera_properties(left_camera_properties):
	global left_K, left_D, left_R, left_P, height, width

	left_K = np.array(left_camera_properties.K).reshape((3,3))

	left_D = np.array(left_camera_properties.D).reshape((-1,1))
	left_R = np.array(left_camera_properties.R).reshape((3,3))
	left_P = np.array(left_camera_properties.P).reshape((3,4))
	height, width = left_camera_properties.height, left_camera_properties.width


def point_cloud_object(point_cloud):
	global cloud

	cloud = point_cloud

def publish_object_pose_tf_rviz(object_points,image_points,frame_names):

	for points3D, points2D, name in zip(object_points,image_points,frame_names):
		rospy.loginfo("No. of object points: %d",len(points3D))
		rospy.loginfo("No. of image points: %d",len(points2D))
		rospy.loginfo("object names: %s",name)
		pose = compute_object_pose(np.array(points3D, dtype="float32"),np.array(points2D,dtype="double"))


def compute_object_pose(object_points, image_points):

	pose = Pose()

	success,rotation,translation,inliers = cv2.solvePnPRansac(object_points, image_points,
																left_K, left_D, flags=cv2.SOLVEPNP_ITERATIVE)

	if success:
		print("Camera rotation vector w.r.t model: {}".format(rotation))
		print("Camera translation vector w.r.t model: {}".format(translation))


	return pose


def publish_marker(points,name):

	#transformation from stereo camera to opencv camera
	tf_opencv_camera_stereo_camera = euler_matrix(-1.5707963,0.0,-1.5707963)

	pose = Point()
	stereo_points = list()

	for point in points:
		#projecting points from opencv_camera frame to stereo_camera frame
		point = np.reshape(np.array(point), (-1,1))

		point_stereo_camera = np.matmul(tf_opencv_camera_stereo_camera,point)
		pose.x = point_stereo_camera[0]
		pose.y = point_stereo_camera[1]
		pose.z = point_stereo_camera[2]

		stereo_points.append(pose)


	marker_points = Marker()

	marker_points.header.frame_id = "stereo_left_camera"
	marker_points.header.stamp = rospy.Time.now()
	marker_points.header.seq = 1
	
	marker_points.ns = name
	marker_points.id = 8
	marker_points.type = marker_points.POINTS
	marker_points.action = marker_points.ADD
	
	marker_points.pose.orientation.w = 1.0
	
	marker_points.scale.x = 0.05
	marker_points.scale.y = 0.05
	#marker_points.scale.z = 0.05
	
	marker_points.color.r = 1.0
	marker_points.color.g = 1.0
	marker_points.color.b = 0.0
	marker_points.color.a = 1.0

	marker_points.points = stereo_points
	#print("All points are transformed and published to visulaize in RVIZ")
	pub_marker.publish(marker_points)

def visulaise_objects_markers_rviz(object_points,object_names):

	for points_3d,name in zip(object_points,object_names):
		publish_marker(points_3d, name)


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


	
def contours_correspondence_3d(contours, objects):
	
	object_3d_points = []
	object_names=[]
	object_2d_points = []

	#homogeneous transformation matrix for the camera w.r.t stereo frame in Arrangement(URDF)
	tf_opencv_camera_stereo_camera = euler_matrix(-1.5707963,0.0,-1.5707963)

	for obj in objects:
		points_xyz=[]
		points_uv=[]
		for point in contours[obj.get("contour")]:
		 	u,v = point[0][0], point[0][1]
			#print("U: {}  V: {}".format(u,v))
		 	x,y,z = x_y_z_from_pcd(u,v) #w.r.t opencv camera frame (z-axis out of the camera)
		 	position_homogeneous = np.reshape(np.array([x,y,z,1]), (-1,1))
		 	#transforming point from opencv_camera frame to stereo_frame (URDF) this is because 
		 	# stereo_image_proc publishes the sparse point cloud data w.r.t opencv camera frame (z-axis out of the camera)
		 	point_stereo_camera = np.matmul(tf_opencv_camera_stereo_camera,position_homogeneous)
		 	print(point_stereo_camera[0:3])

		 	if not(math.isnan(x) or math.isnan(y) or math.isnan(z)):
		 		#print("Checked for nan and appending")
				points_xyz.append(point_stereo_camera[0:3])
				points_uv.append([u,v])
		
		object_3d_points.append(points_xyz)
		object_names.append(obj.get("name"))
		object_2d_points.append(points_uv)


	return (object_3d_points, object_2d_points, object_names)


def detect_objects_contours(left_image):

	gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)


	left_masked = cv2.adaptiveThreshold(gray_left,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,7,2)

	left_masked = cv2.medianBlur(left_masked,3)

	image, contours,heirarchy = cv2.findContours(left_masked,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	area_centroid = []
	
	for i in range(len(contours)):
		moments = cv2.moments(contours[i])

		if moments['m00'] > 0:
			cx = cx = int(moments['m10']/moments['m00'])
			cy = int(moments['m01']/moments['m00']) 
			area = moments['m00']
			#considering only the cube and cylinder objects in the scene (area is tuned)
			if (area > 1400 and area < 1800):
				area_centroid.append({"area":area,"cx":cx,"cy":cy,"contour":i,"name":"Cuboid"})
			if (area > 4000 and area < 5000):
				area_centroid.append({"area":area,"cx":cx,"cy":cy,"contour":i,"name":"Cylinder"})

	for cnt in area_centroid:

		contour_points = len(contours[cnt.get("contour")])

		cv2.drawContours(left_image, contours, cnt.get("contour"), (0,255,0), 3)
		cv2.imshow("gray_left", left_masked)
		cv2.imshow("gray_left_raw", left_image)
		cv2.waitKey(3)

	return (contours,area_centroid)

#function that initates the subscription of topics
def image_proc():
	global pub_marker

	left_cam = rospy.Subscriber("/stereo/left/camera_info", CameraInfo, callback=left_camera_properties)
	right_cam = rospy.Subscriber("/stereo/right/camera_info", CameraInfo, callback=right_camera_properties)
	left_image = rospy.Subscriber("/stereo/left/image_rect_color", Image, callback=left_image_rect)
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

		object_contours, objects = detect_objects_contours(left_image)

		object_points,image_points,object_names = contours_correspondence_3d(object_contours,objects)

		print(len(object_points),len(image_points))

		time.sleep(10)

		if (len(object_points) == len(image_points)) and (len(object_points) > 0):

			publish_object_pose_tf_rviz(object_points,image_points,object_names)
		else:
			rospy.logwarn("No onjects detected")

	
	print("Shutting down")
	cv2.destroyAllWindows()

main()