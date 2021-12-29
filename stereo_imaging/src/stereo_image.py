#! /usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from geometry_msgs.msg import Point,Pose
import numpy as np
from visualization_msgs.msg import Marker
from manipulation_msgs.msg import ObjectData
from tf.transformations import euler_matrix, euler_from_quaternion, quaternion_from_euler, quaternion_from_matrix
import tf
import math
import time
import struct

bridge_image = CvBridge() #instance to bridge the ros image to cv2 image

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
		
		assert len(points3D)==len(points2D), "object points and image points should be of same length"
		

		#sample 21 points
		if len(points3D) >=21:
			sample_points_indexes = np.random.choice(len(points3D),21,replace=False)
			points3D_sampled = np.array([points3D[i] for i in sample_points_indexes],dtype="float32")
			
			#setting the frame of the pose at the bottom of the object 
			points3D_sampled_tf = np.array([points3D[i] - points3D[len(points3D)//2] for i in sample_points_indexes])
			points2D_sampled = np.array([points2D[i] for i in sample_points_indexes],dtype="double")
		else:
			rospy.loginfo("No enough points for the pose computation...")
			return
		
		pose, success = compute_object_pose(points3D_sampled_tf,points2D_sampled,name)

		if success:
			transformer = tf.TransformBroadcaster()
			transformer.sendTransform(pose.position, pose.orientation,rospy.Time(),name,"/stereo_base")




def compute_object_pose(object_points, image_points,object_name):

	pose = Pose()

	success,rotation,translation = cv2.solvePnP(object_points, image_points,
		left_K, left_D, flags=cv2.SOLVEPNP_ITERATIVE)

	#Transforming the Model pose from opencv camera frame to gazebo camera frame if available
	if success:

		euler_angle = rotation.reshape(-1)
		object_pose = euler_matrix(euler_angle[0],euler_angle[1],euler_angle[2],axes="sxyz")
		object_pose[:3,3] = translation.reshape(-1)


		tf_opencv_camera_stereo_camera = euler_matrix(-1.5707963,0.0,-1.5707963)
		
		#from opencv camera frame to the gazebo camera frame (because x-axis is out of the camera in gazebo 
		#while z-axis is out in opencv camera)
		pose_gazebo = np.matmul(tf_opencv_camera_stereo_camera,object_pose)

		#pose of the object w.r.t global frame (location in world)
		pose_world_translation, pose_world_rotation = object_pose_world(pose_gazebo)

		print("Pose of the object in the world: \n Object name: {}".format(object_name))
		print("Rotation: ")
		print(euler_from_quaternion(pose_world_rotation))
		print("Translation: ")
		print(pose_world_translation)

		pose.position = pose_world_translation
		pose.orientation = pose_world_rotation

		#############Test for the projection of object points to image points###############		

		# ext = np.delete(object_pose,3,0)
		# print(ext.shape)
		# print(ext)
		# point3D = np.append(object_points[5],1).reshape((-1,1)) #choosing a random point from the samples or any other
																# point whose image co-ords are known
		# print(point3D)
		# image = np.matmul(left_K, np.matmul(ext,point3D)) #assuming no distortion in the lens

		# print(image)

		# image = np.array([int(image[0]/image[2]),int(image[1]/image[2])]) #converting homogeneous image coords to pixel coords

		# print("Estimated",image)
		# print("Actual",image_points[5])
		# print(image == image_points[5])

		#True indicates that the model pose computed with solvePnP is a good approximate to the object pose

		#####################################################################################

		return pose,True

	return pose,False

def object_pose_world(pose_gazebo):

	source_frame = "/stereo_left_camera"
	target_frame = "/stereo_base"
	time = rospy.Time()
	
	t = tf.TransformListener()
	t.waitForTransform(target_frame,source_frame,time,rospy.Duration(5))
	camera_world_t, camera_world_r = t.lookupTransform(target_frame, source_frame, time)
	#print("Translation w.r.t base")
	#print(camera_world_t)
	object_pose_world = np.matmul(t.fromTranslationRotation(camera_world_t, camera_world_r),pose_gazebo)

	translation_vector = object_pose_world[:3,3]

	rotation_matrix = np.zeros((4,4))
	rotation_matrix[:3,:3] = object_pose_world[:3,:3]
	rotation_matrix[3,3] = 1
	rotation_vector = quaternion_from_matrix(rotation_matrix)

	return translation_vector, rotation_vector


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


def publish_object_data(names,height):
	obj = ObjectData()
	obj.height = height
	obj.name = names
	pub_data.publish(obj)


def contours_correspondence_3d(contours, objects):
	
	object_3d_points = []
	object_names=[]
	object_2d_points = []
	height_object = []
	#transformation matrix for the camera w.r.t stereo frame in Arrangement(URDF)
	tf_opencv_camera_stereo_camera = euler_matrix(-1.5707963,0.0,-1.5707963)

	#for each objects detected get the corresponding xyz coordinates related to pixel coordinates from the contour 
	for obj in objects:
		points_xyz=[]
		points_uv=[]
		for point in contours[obj.get("contour")]:
		 	u,v = point[0][0], point[0][1]
			
		 	x,y,z = x_y_z_from_pcd(u,v) #w.r.t opencv camera frame (z-axis out of the camera)
		 	position_homogeneous = np.reshape(np.array([x,y,z,1]), (-1,1))
		 	
		 	if not(math.isnan(x) or math.isnan(y) or math.isnan(z)):
				points_xyz.append(position_homogeneous[0:3])
				points_uv.append([u,v])
		
		#height will be used mainly when grasping of an object is necessary
		height_object.append(abs(np.max(np.array(points_xyz)[:,1,0]) - np.min(np.array(points_xyz)[:,1,0]))) #the value along y in opencv
		
		object_3d_points.append(points_xyz)
		object_names.append(obj.get("name"))
		object_2d_points.append(points_uv)


	return (object_3d_points, object_2d_points, object_names, height_object)


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
			#considering only the cube and cylinder objects in the scene (area is hand tuned)
			if (area > 950 and area < 1300):
				#print(area)
				area_centroid.append({"area":area,"cx":cx,"cy":cy,"contour":i,"name":"Cylinder"})
			
			if (area > 5000 and area < 6000):
				#print("Cuboid")
				area_centroid.append({"area":area,"cx":cx,"cy":cy,"contour":i,"name":"Cuboid"})

	for cnt in area_centroid:

		contour_points = len(contours[cnt.get("contour")])

		cv2.drawContours(left_image, contours, cnt.get("contour"), (0,255,0), 3)
		cv2.imshow("gray_left", left_masked)
		cv2.imshow("gray_left_raw", left_image)
		cv2.waitKey(1)

	return (contours,area_centroid)

#function that initates the subscription of topics
def image_proc():
	global pub_data

	left_cam = rospy.Subscriber("/stereo/left/camera_info", CameraInfo, callback=left_camera_properties)
	right_cam = rospy.Subscriber("/stereo/right/camera_info", CameraInfo, callback=right_camera_properties)
	left_image = rospy.Subscriber("/stereo/left/image_raw", Image, callback=left_image_rect)
	right_image = rospy.Subscriber("/stereo/right/image_raw", Image, callback=right_image_raw)

	pcd_3d = rospy.Subscriber("/stereo/points2",PointCloud2,point_cloud_object)

	pub_data = rospy.Publisher("/object/data",ObjectData,queue_size=1)

def main():
	
	rospy.init_node("stereo_imaging")
	print("Node Started")
	image_proc()
	print("Waiting for the RGB image....")
	rospy.wait_for_message("/stereo/left/image_raw",Image)
	print("RGB Image available....")
	print("Waiting for the point cloud...")
	rospy.wait_for_message("/stereo/points2",PointCloud2)
	print("Point cloud received.....")

	while not rospy.is_shutdown():

		object_contours, objects = detect_objects_contours(left_image)

		object_points,image_points,object_names,height = contours_correspondence_3d(object_contours,objects)

		#print(len(object_points),len(image_points))

		if (len(object_points) == len(image_points)) and (len(object_points) > 0):
			publish_object_pose_tf_rviz(object_points,image_points,object_names)
			publish_object_data(object_names,height)
		
		else:
			rospy.logwarn("No objects detected")

	
	print("Shutting down")
	cv2.destroyAllWindows()

main()