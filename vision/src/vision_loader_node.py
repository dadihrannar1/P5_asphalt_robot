#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tf_convert
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PointStamped
#import geometry_msgs.msg as geo_msgs
import nav_msgs.msg as nav_msgs
from std_msgs.msg import Float32
from webots_ros.srv import get_float
import math
import time
import pickle
from vision.srv import Display_input, Display_inputRequest

vehicle_speed = 0.0
def vehicle_vel_callback(msg):
    global vehicle_speed
    vehicle_speed = msg.data
    print(f"Vision: vehicle speed is {vehicle_speed}")

# Function for adding angles bounded to [0, 2*PI[
def angle_add(angle_1, angle_2):
    if(angle_1 + angle_2 >= 2*math.pi):
        return angle_1 + angle_2 - 2*math.pi
    elif(angle_1 + angle_2 < 0):
        return angle_1 + angle_2 + 2*math.pi
    else:
        return angle_1 + angle_2

# Function to transform a coordinate into world coordinates
def transform_coordinates(x_coordinate, y_coordinate, transformation):
    # Pre-allocate the point and result arrays
    point = np.zeros((4, 1))

    # Assign values to the point array
    point[0, 0] = x_coordinate
    point[1, 0] = y_coordinate
    point[3, 0] = 1

    # Transform the coordinate
    result = np.dot(transformation, point)

    # Unpack the result into x and y variables
    x, y, _, _ = result
    return x, y

def vision_pub(filenames, paths, timestamps, offsets, image_folder_path):
    point_pub = rospy.Publisher('/points', PointStamped, queue_size=10)
    transform_pub = rospy.Publisher('/vo', nav_msgs.Odometry, queue_size=50)
    vehicle_vel_sub = rospy.Subscriber("/velocity", Float32, vehicle_vel_callback)
    simulation_time_client = rospy.ServiceProxy("/fivebarTrailer/robot/get_time", get_float)

    start_image = rospy.get_param("~Start_image")
    end_image = rospy.get_param("~End_image")
    if(type(end_image) != int):
        exit("Vision_node: WRONG LAUNCH INPUT TYPE")
    if(end_image<start_image):
        exit("Vision_node: Invalid launch input end<start")

    r = rospy.Rate(10000)  # 100hz used for sending coordinates

    # Transform listener for getting transforms from TF
    tf_buffer = tf2_ros.Buffer(rospy.Time(10000))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    PIXEL_SIZE = 0.0008853*2   # In meters

    # Transform from camera to base (must be the inverse of base_to_camera_transform)
    transform_camera_to_base = Transform()
    transform_camera_to_base.translation.x = 1080/2*PIXEL_SIZE/2
    transform_camera_to_base.translation.y = -0.0584708
    transform_camera_to_base.translation.z = 0
    transform_camera_to_base.rotation.x = 0.7071068
    transform_camera_to_base.rotation.y = 0.7071068
    transform_camera_to_base.rotation.z = 0
    transform_camera_to_base.rotation.w = 0

    quaternion = (transform_camera_to_base.rotation.x, transform_camera_to_base.rotation.y, transform_camera_to_base.rotation.z, transform_camera_to_base.rotation.w)
    translation = (transform_camera_to_base.translation.x, transform_camera_to_base.translation.y, transform_camera_to_base.translation.z)
    transform_camera_to_base_matrix = tf_convert.quaternion_matrix(quaternion)
    transform_camera_to_base_matrix[:3, 3] = translation

    camera_to_base_rotation_only = np.zeros((4,4))
    camera_to_base_rotation_only[0:3, 0:3] = transform_camera_to_base_matrix[0:3, 0:3]
    camera_to_base_rotation_only[3,3] = 1


    world_pose_x = 0.0584708
    world_pose_y = -0.3381208
    world_orientation = 0.0

    # Scalar from pixels to distances in camera frame
    STANDARD_COVARIANCE = [1.9074e-05, 0, 0, 0, 0, 0,
                           0, 1.9074e-05, 0, 0, 0, 0,
                           0, 0, 1.9074e-05, 0, 0, 0,
                           0, 0, 0, 1.9074e-05, 0, 0,
                           0, 0, 0, 0, 1.9074e-05, 0,
                           0, 0, 0, 0, 0, 1.9074e-05]

    # Publish initial position to tf
    tf_msg = nav_msgs.Odometry()
    tf_msg.header.stamp.secs = 1
    tf_msg.header.stamp.nsecs = int(00000000)
    tf_msg.header.frame_id = "world_frame"
    tf_msg.child_frame_id = "vo"
    tf_msg.twist.twist.linear.x = 0.0
    tf_msg.twist.twist.linear.y = 0.0
    tf_msg.twist.twist.linear.z = 0.0
    tf_msg.twist.twist.angular.x = 0.0
    tf_msg.twist.twist.angular.y = 0.0
    tf_msg.twist.twist.angular.z = 0.0
    tf_msg.twist.covariance = STANDARD_COVARIANCE
    tf_msg.pose.pose.position.x = 0.0
    tf_msg.pose.pose.position.y = 0.0
    tf_msg.pose.pose.position.z = 0.0
    quat = tf_convert.quaternion_from_euler(0, 0, 0)
    tf_msg.pose.pose.orientation.x = quat[0]
    tf_msg.pose.pose.orientation.y = quat[1]
    tf_msg.pose.pose.orientation.z = quat[2]
    tf_msg.pose.pose.orientation.w = quat[3]
    tf_msg.pose.covariance = STANDARD_COVARIANCE
    #print(tf_msg)
    transform_pub.publish(tf_msg)

    rospy.wait_for_service('/input_display')
    #simulation_time_client.wait_for_service()

    # find the range by matching start and end image index
    start_index = ''
    end_index = ''
    for i, filename in enumerate(filenames):
        image_number = int(filename.split('saved_image_')[-1].split('.png')[0])
        if image_number == start_image:
            start_index = i
        if image_number == end_image:
            end_index = i
    if type(start_index) == str or type(end_index) == str:
        exit("Vision_node: Specified image range is outside pickled range")

    # Wait for the ekf to be available
    #rospy.wait_for_service('robot_pose_ekf/get_status')

    # Start the image stitcher with the same path as the images loaded in the vision node
    request = Display_inputRequest()
    request.path = image_folder_path # Path must contain json file and images
    request.start_image = start_image # first image number
    request.amount_of_images = end_image - start_image # number of images (max ~60)
    #print(request.path,request.start_image,request.amount_of_images)

    print("Vision: Sending images to the simulation\n")
    image_stitch = rospy.ServiceProxy('/input_display', Display_input)
    image_stitch.call(request)

    print('Started vision_pub')
    for i in range(start_index, end_index):
        # Fetch trajectory
        path = paths[i]
        timestamp = timestamps[i]
        angle, traveled_x, traveled_y = offsets[i]  # swapped x and y to move in right direction TODO: make this change earlier in the pipeline

        # Wait for webots timing
        previous_time = simulation_time_client.call(True).value
        #rint(f"Vision: Time before waiting = {previous_time}")
        if vehicle_speed == 0.0:
            # Wait to get first image for as long as the arduino recorded
            #print(f"Vision: Time to wait = {timestamp/1000}seconds")
            while simulation_time_client.call(True).value < previous_time + timestamp/1000:
                #curr_time = simulation_time_client.call(True)
                time.sleep(0.008)
                pass
            previous_time = simulation_time_client.call(True).value + timestamp/1000
        else:
            # Convert encoder ticks and desired vehicle speed to wait time for next image
            time_to_next_image = traveled_y*PIXEL_SIZE / vehicle_speed

            # Wait to get first image for as long as the vehicle velocity demands
            while simulation_time_client.call(True).value < previous_time + time_to_next_image:
                #curr_time = simulation_time_client.call(True)
                time.sleep(0.008)
                pass
            previous_time = simulation_time_client.call(True).value + time_to_next_image

        # Get timestamp for tf
        secs = simulation_time_client.call(True).value
        #print(f"Vision: Time after waiting = {secs}\n--------------------------------\n")

        #print(f"Vision: X_travel = {traveled_x}, Y_travel = {traveled_y}")
        
        # Calculate world pose
        traveled_x = 0 #TODO: fix the incorrect translation and rotation from the image aligner
        angle = 0
        #world_pose_x += (traveled_x * math.cos(world_orientation) + traveled_y * math.sin(world_orientation)) * PIXEL_SIZE
        #world_pose_y += (traveled_x * math.sin(world_orientation) + traveled_y * math.cos(world_orientation)) * PIXEL_SIZE
        world_pose_x += traveled_x * PIXEL_SIZE
        world_pose_y += traveled_y * PIXEL_SIZE
        world_orientation = angle_add(world_orientation, -angle)

        # Transform everything from camera frame to base frame
        traveled_x_base, traveled_y_base = transform_coordinates(traveled_x * PIXEL_SIZE, traveled_y * PIXEL_SIZE, camera_to_base_rotation_only)
        world_pose_x_base, world_pose_y_base = transform_coordinates(world_pose_x, world_pose_y, transform_camera_to_base_matrix)

        #print(f"Vision: X_travel = {traveled_x_base}, Y_travel = {traveled_y_base}")
        #print(f"Vision: X_pos = {world_pose_x_base}, Y_pos = {world_pose_y_base}")

        # Convert from seconds (float) to seconds and nanoseconds
        nanoseconds = int(secs*1e9%1e9)
        seconds = int(secs*1e9//1e9)

        # Publish transform from camera
        tf_msg = nav_msgs.Odometry()
        tf_msg.header.stamp.secs = seconds
        tf_msg.header.stamp.nsecs = nanoseconds
        tf_msg.header.frame_id = "world_frame"
        tf_msg.child_frame_id = "vo"

        # Add twist
        tf_msg.twist.twist.linear.x = traveled_x_base
        tf_msg.twist.twist.linear.y = traveled_y_base
        tf_msg.twist.twist.linear.z = 0.0
        tf_msg.twist.twist.angular.x = 0.0
        tf_msg.twist.twist.angular.y = 0.0
        tf_msg.twist.twist.angular.z = angle
        tf_msg.twist.covariance = STANDARD_COVARIANCE

        # Add pose
        tf_msg.pose.pose.position.x = world_pose_x_base
        tf_msg.pose.pose.position.y = world_pose_y_base
        tf_msg.pose.pose.position.z = 0.0
        quat = tf_convert.quaternion_from_euler(0, 0, world_orientation)
        tf_msg.pose.pose.orientation.x = quat[0]
        tf_msg.pose.pose.orientation.y = quat[1]
        tf_msg.pose.pose.orientation.z = quat[2]
        tf_msg.pose.pose.orientation.w = quat[3]
        tf_msg.pose.covariance = STANDARD_COVARIANCE
        #print(tf_msg)
        transform_pub.publish(tf_msg)
        rospy.sleep(0.01)
        #DEBUG
        #print(f"Vision stamp: sec {tf_msg.header.stamp.secs}, nsec {tf_msg.header.stamp.nsecs}\n")

        
        # TODO: EKF is a few (1-2) transforms behind which causes extrapolation errors into the future
        #while(not tf_buffer.can_transform('world_frame', 'camera_frame', rospy.Time(seconds, nanoseconds))):
        #    #print("vision waiting for transform")
        #    time.sleep(0.1)
        #    pass

        # Get transform from camera to world for current image
        # print(f"Vision: Transform available")
        try:
            transform_camera_to_world = tf_buffer.lookup_transform('world_frame', 'camera_frame', rospy.Time(seconds, nanoseconds))
            # Send each point in crack trajectory
            #print(f'path generated: {path}')
            
            message = PointStamped()
            message.header.frame_id = "world_frame"
            message.header.stamp.secs = seconds
            message.header.stamp.nsecs = nanoseconds

            # Create transformation matrix
            quaternion = (transform_camera_to_world.transform.rotation.x, transform_camera_to_world.transform.rotation.y, transform_camera_to_world.transform.rotation.z, transform_camera_to_world.transform.rotation.w)
            translation = (transform_camera_to_world.transform.translation.x, transform_camera_to_world.transform.translation.y, transform_camera_to_world.transform.translation.z)
            point_transformation_matrix = tf_convert.quaternion_matrix(quaternion)
            point_transformation_matrix[:3, 3] = translation

            for x, y, _, _ in path:
                # Use tuple unpacking to assign values to x and y
                coords = transform_coordinates(x * PIXEL_SIZE, y * PIXEL_SIZE, point_transformation_matrix)
                
                message.point.x = coords[0]
                message.point.y = coords[1]
                #print(message)
                point_pub.publish(message)
                r.sleep()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exception:
            print(f"Vision: {exception}")
            # For first transform publish again to ensure transform is available for first point
            #r.sleep()
            continue

if __name__ == "__main__":
    rospy.init_node('vision_publisher')


    # load pickle file
    image_path = rospy.get_param("~Image_path")
    with open(f'{image_path}/vision_output.pkl', 'rb') as f:
        filenames_from_pkl = pickle.load(f)   # list of image numbers
        paths_from_pkl = pickle.load(f)       # list of paths (list of points)
        timestamps_from_pkl = pickle.load(f)  # list of milliseconds
        offsets_from_pkl = pickle.load(f)     # list of (angle, traveled_x, traveled_y)
    vision_pub(filenames_from_pkl, paths_from_pkl, timestamps_from_pkl, offsets_from_pkl, image_path)