#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tf_convert
import geometry_msgs.msg as geo_msgs
import nav_msgs.msg as nav_msgs
from std_msgs.msg import Float64
from webots_ros.srv import get_float
import math
import time
import pickle
from vision.srv import Display_input, Display_inputRequest

vehicle_speed = ''
def vehicle_vel_callback(msg):
    vehicle_speed = msg.data

# Function for adding angles bounded to [0, 2*PI[
def angle_add(angle_1, angle_2):
    if(angle_1 + angle_2 >= 2*math.pi):
        return angle_1 + angle_2 - 2*math.pi
    elif(angle_1 + angle_2 < 0):
        return angle_1 + angle_2 + 2*math.pi
    else:
        return angle_1 + angle_2

# Function to transform a coordinate into world coordinates
def transform_coordinates(x_coordinate, y_coordinate, transform: geo_msgs.Transform):
    
    if not isinstance(transform, geo_msgs.Transform):
        raise TypeError
    else:
        # 4x4 transformation matrix from quaternion and translation
        quaternion = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
        transformation = tf_convert.quaternion_matrix(quaternion)
        transformation[0, 3] = transform.translation.x
        transformation[1, 3] = transform.translation.y
        transformation[2, 3] = transform.translation.z

        # Crack coordinate as pose
        point = np.array([[x_coordinate], [y_coordinate], [0], [1]])

        # Transform coordinate
        result = np.dot(transformation, point)
        x = result[0, 0]
        y = result[1, 0]
        return x, y

def vision_pub(filenames, paths, timestamps, offsets):
    point_pub = rospy.Publisher('/points', geo_msgs.PointStamped, queue_size=10)
    transform_pub = rospy.Publisher('/vo', nav_msgs.Odometry, queue_size=50)
    vehicle_vel_sub = rospy.Subscriber("/vehicle_speed", Float64, vehicle_vel_callback)
    simulation_time_client = rospy.ServiceProxy(f"/fivebarTrailer/robot/get_time", get_float)

    start_image = rospy.get_param("~Start_image")
    end_image = rospy.get_param("~End_image")
    if(type(end_image) != int):
        exit("Vision_node: WRONG LAUNCH INPUT TYPE")
    if(end_image<start_image):
        exit("Vision_node: Invalid launch input end<start")

    r = rospy.Rate(100)  # 100hz used for sending coordinates

    # Transform listener for getting transforms from TF
    tf_buffer = tf2_ros.Buffer(rospy.Time(100))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Transform from camera to base (must be the inverse of base_to_camera_transform)
    transform_camera_to_base = geo_msgs.Transform()
    transform_camera_to_base.translation.x = -0.067
    transform_camera_to_base.translation.y = 0.42665
    transform_camera_to_base.translation.z = 0
    transform_camera_to_base.rotation.x = 0
    transform_camera_to_base.rotation.y = 0
    transform_camera_to_base.rotation.z = 0
    transform_camera_to_base.rotation.w = 1

    world_pose_x = 0
    world_pose_y = 0
    world_orientation = 0

    # Scalar from pixels to distances in camera frame
    PIXEL_SIZE = 0.0009712  # In meters
    STANDARD_COVARIANCE = [1.9074e-05, 0, 0, 0, 0, 0,
                           0, 1.9074e-05, 0, 0, 0, 0,
                           0, 0, 1.9074e-05, 0, 0, 0,
                           0, 0, 0, 1.9074e-05, 0, 0,
                           0, 0, 0, 0, 1.9074e-05, 0,
                           0, 0, 0, 0, 0, 1.9074e-05]

    simulation_time_client.wait_for_service()

    # find the range by matching start and end image index
    start_index = ''
    end_index = ''
    for i, filename in enumerate(filenames):
        print(f"Vision: filename: {filename}")
        print(f"Vision: filetype: {type(filename)}")
        image_number = int(filename.split('saved_image_')[-1].split('.png')[0])
        if image_number == start_image:
            start_index = i
        if image_number == end_image:
            end_index = i
    if type(start_index) == str or type(end_index) == str:
        exit("Vision_node: Specified image range is outside pickled range")

    # Start the image stitcher with the same path as the images loaded in the vision node
    request = Display_inputRequest()
    request.path = filenames # Path must contain json file and images
    request.start_image = start_image # first image number
    request.amount_of_images = end_image - start_image # number of images (max ~60)

    print("Thread 1: Sending images to the simulation\n")
    image_stitch = rospy.ServiceProxy('/input_display', Display_input)
    image_stitch.call(request)

    print('Started vision_pub')
    for i in range(start_index, end_index):
        # Fetch trajectory
        path = paths[i]
        timestap = timestamps[i]
        angle, traveled_x, traveled_y = offsets[i, 2]

        # Wait for webots timing
        previous_time = simulation_time_client.call(True)
        previous_time = previous_time.value
        if type(vehicle_speed) == str:
            # Wait to get first image for as long as the arduino recorded
            curr_time = simulation_time_client.call(True)
            while curr_time.value > previous_time + timestap[i]/1000:
                time.sleep(0.1)
            previous_time = curr_time.value + timestap[i]/1000
        else:
            # Convert encoder ticks and desired vehicle speed to wait time for next image
            time_to_next_image = offsets[i+1, 1] / vehicle_speed  #THIS MAY FUCK UP AND MAKE THE TIME TO WAIT VERY SHORT IF IT DOES CHANGE INDEX TO 2
            
            # Wait to get first image for as long as the vehicle velocity demands
            while simulation_time_client.call(True).value > previous_time + time_to_next_image:
                time.sleep(0.1)
            previous_time = simulation_time_client.call(True).value + time_to_next_image

        # Get timestamp for tf
        secs = simulation_time_client.call(True).value

        # Calculate world pose
        world_pose_x += traveled_x * PIXEL_SIZE
        world_pose_y += traveled_y * PIXEL_SIZE
        world_orientation = angle_add(world_orientation, angle)

        # Transform everything from camera frame to base frame
        traveled_x_base, traveled_y_base = transform_coordinates(traveled_x, traveled_y, transform_camera_to_base)
        world_pose_x_base, world_pose_y_base = transform_coordinates(world_pose_x, world_pose_y, transform_camera_to_base)

        # Publish transform from camera
        tf_msg = nav_msgs.Odometry()
        tf_msg.header.stamp.from_sec(secs)
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

        transform_pub.publish(tf_msg)

        #DEBUG
        print("Vision sent a transform to tf\n")

        while(not tf_buffer.can_transform('world_frame', 'camera_frame')):
            print("vision waiting for transform")
            time.sleep(1)

        # Get transform from camera to world for current image
        try:
            transform_camera_to_world = tf_buffer.lookup_transform('world_frame', 'camera_frame', rospy.Time.from_sec(secs))

            # Send each point in crack trajectory
            for point in path:
                # geometry_msgs PointStamped Message
                message = geo_msgs.PointStamped()
                message.header.frame_id = "world_frame"
                message.header.stamp.from_sec(secs)
                coords = transform_coordinates(point[0] * PIXEL_SIZE, point[1] * PIXEL_SIZE, transform_camera_to_world.transform)
                message.point.x = coords[0]
                message.point.y = coords[1]
                # Used for sending the end of crack information
                message.point.z = point[2]

                point_pub.publish(message)
                r.sleep()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exception:
            print(exception)
            # For first transform publish again to ensure transform is available for first point
            r.sleep()
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

    vision_pub(filenames_from_pkl, paths_from_pkl, timestamps_from_pkl, offsets_from_pkl)