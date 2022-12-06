#!/usr/bin/env python3

import numpy as np
import cv2
import time
import math
import random
import rospy
import tf2_ros
import tf.transformations as tf_convert
import torch
import albumentations
from albumentations.pytorch.transforms import ToTensorV2
from multiprocessing import Process, Lock, Event
from multiprocessing.managers import BaseManager
import copy
from os.path import exists
from pathlib import Path
from model_utils import load_model
import image_aligner
from trajectory_planning import Crack, Frame, map_cracks, process_image
import geometry_msgs.msg as geo_msgs
import nav_msgs.msg as nav_msgs
import atexit
import pickle
import glob


# Camera source (0 for webcam)
capture_src = 0

# Path to model
current_path = str(Path(__file__).parent)
model_name = 'crack500v4.pth.tar'

# Processing device for running model
DEVICE = 'cpu'

# Parameters
HEIGHT = 320
WIDTH = 480


# Class structure for tranferring data between the processes
class DataTransfer:
    def __init__(self) -> None:
        self.data = 0
        self.offset_x = 0
        self.offset_y = 0
        self.offset_rot = 0
        self.frame_time = 0

    def set_data(self,value):
        self.data = value

    def get_data(self):
        return self.data

    def set_image_offset(self, x_trans, y_trans, z_rot):
        self.offset_x = x_trans
        self.offset_y = y_trans
        self.offset_rot = z_rot

    def get_image_offset(self):
        return self.offset_x, self.offset_y, self.offset_rot

    def set_frame_time(self, value):
        self.frame_time = value

    def get_frame_time(self):
        return self.frame_time


##Functions to acquire images
# TODO hook this up to the actual camera
def frames_from_camerastream(data_out, lock, event):
    print('Starting video capture')
    cap = cv2.VideoCapture(capture_src)

    # Image capture loop
    while True:
        # Try to acquire the first frame
        if cap.isOpened():
            rval, frame = cap.read()
            print(rval)
        else:
            rval = False
            print('No video stream')
            time.sleep(10) # wait 10 seconds for video stream to open
        
        while rval:
            frame_time = time.time_ns()
            rval, frame = cap.read()

            lock.acquire()
            data_out.set_data(frame)
            data_out.set_frame_time(frame_time)
            event.set()
            lock.release()


def frames_from_files(data_out, lock, event_transmit, event_transmit_ready):
    with open(current_path + '/calib_matrix.pkl', 'rb') as file:
        map_x = pickle.load(file)
        map_y = pickle.load(file)

    image_path = current_path + '/images/*.png'
    images = sorted(glob.glob(image_path))
    print('Thread 1 started (frames_from_files)')
    for filename in images:
        frame_time = time.time_ns()
        frame = cv2.imread(filename)

        # Undistort image, followed by rotation and cropping
        frame_corrected = cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)
        frame_rotated = cv2.rotate(frame_corrected, cv2.ROTATE_90_CLOCKWISE)
        frame_cropped = frame_rotated[100:1720, 100:980]  # Crop image to remove trailer edges
        #visualize = cv2.resize(frame_cropped, [320, 480])
        #cv2.imshow('corrected', visualize)
        #cv2.waitKey(0)

        # Wait for next thread to be ready to recieve
        event_transmit_ready.wait()
        event_transmit_ready.clear()

        # Send frame and time recorded
        lock.acquire()
        data_out.set_data(frame_cropped)
        data_out.set_frame_time(frame_time)
        lock.release()

        # Notify next frame of available data
        event_transmit.set()
        #print('Thread 1 sent data')


##Function to run model from specifed path
# Run as a seperate thread for the program
def run_model(data_in, data_out, lock_in, lock_out, event_transmit, event_transmit_ready, event_receive, event_receive_ready):
    # Load trained model ffrom specifed path
    model = load_model(current_path + '/' + model_name)

    # Transforms
    detect_transform = albumentations.Compose([
                albumentations.Resize(height=HEIGHT, width=WIDTH),
                albumentations.Normalize(
                    mean=[0.0, 0.0, 0.0],
                    std=[1.0, 1.0, 1.0],
                    max_pixel_value=255.0,
                ),
                ToTensorV2()])

    # Set alignment values for first runthrough where there is no previous image alignment
    img_raw_old = np.zeros(([2, 2]), dtype=np.uint8)
    local_image = 0
    traveled_x = 0
    traveled_y = 0
    angle = 0

    event_receive_ready.set()

    # Main thread loop
    print('Thread 2 started (run_model)')
    while(True):
        event_receive.wait() # Wait for event flag from the image acquisition thread
        event_receive.clear() # Clear event flag before processing starts (allows a new flag to arrive before processing finishes)

        # Lock data while it is being read by the thread
        lock_in.acquire()
        local_image = data_in.get_data()
        local_frame_time = data_in.get_frame_time()
        lock_in.release()

        event_receive_ready.set()
        #print('Thread 2 got data')
        
        # 
        if img_raw_old.any():
            new_image = cv2.resize(local_image,(WIDTH,HEIGHT),interpolation=cv2.INTER_AREA)
            new_image = cv2.cvtColor(new_image,cv2.COLOR_BGR2GRAY)

            transform = image_aligner.image_aligner_cpu(img_raw_old, new_image)
            angle, traveled_x, traveled_y = image_aligner.affine_to_angle_trans(transform)

        augmentented = detect_transform(image=local_image)
        data = augmentented["image"].to(device=DEVICE)
        data = torch.unsqueeze(data,0)
        output = torch.sigmoid(model(data))
        output = torch.squeeze(output)
        preds = (output > 0.5).float()
        
        #
        img_raw_old = np.copy(local_image)
        img_raw_old = cv2.resize(img_raw_old,(480,320),interpolation=cv2.INTER_AREA)
        img_raw_old = cv2.cvtColor(img_raw_old,cv2.COLOR_BGR2GRAY)
        
        event_transmit_ready.wait()
        event_transmit_ready.clear()

        # Send data to path planning thread
        lock_out.acquire()
        data_out.set_data(preds.cpu().numpy())
        data_out.set_frame_time(local_frame_time)
        data_out.set_image_offset(angle, traveled_x, traveled_y)
        lock_out.release()

        event_transmit.set()
        #print('Thread 2 sent data')


def path_planning(data_in, data_out, lock_in, lock_out, event_transmit, event_transmit_ready, event_receive, event_receive_ready):
    img_old_seg = np.zeros(([2, 2]), dtype=np.uint8)
    old_frame = 0
    traveled = 0

    event_receive_ready.set()

    # Main thread loop
    print('Thread 3 started (path_planning)')
    while True:
        event_receive.wait()    # Wait for new image
        event_receive.clear()   # Clear event
        
        # Get data if lock is free
        lock_in.acquire()
        local_img = data_in.get_data().astype(np.uint8) # get data
        local_frame_time = data_in.get_frame_time()
        angle, traveled_x, traveled_y = data_in.get_image_offset()
        lock_in.release()

        event_receive_ready.set()
        #print('Thread 3 got data')

        sorted_cracks = process_image(local_img)
        
        img_old_seg = local_img
        frame1 = Frame()    
        frame1.set_frame_time(local_frame_time)

        for crack in sorted_cracks:
            frame1.add_crack(Crack(crack))

        # If not first frame, ensure all cracks which have already been marked for repair are not marked again
        if not old_frame == 0:
            map_cracks(old_frame, frame1, traveled_y)
        
        frame1.find_path()
       
        old_frame = copy.copy(frame1)

        event_transmit_ready.wait()
        event_transmit_ready.clear()
        
        # Set data if lock is free
        lock_out.acquire()
        data_out.set_data(frame1)
        data_out.set_frame_time(local_frame_time)
        data_out.set_image_offset(angle, traveled_x, traveled_y)
        lock_out.release()

        event_transmit.set()
        #print('Thread 3 sent data')
        
        #frame0Vis = visualize(frame1, frame1.path,320*0.75)
        #cv2.imshow("crack visualisation", frame0Vis)
        #cv2.waitKey(10)

#Visualise cracks for debugging
def visualize(frame: Frame, p1, offset):
    blank_image = np.zeros((HEIGHT,WIDTH,3), np.uint8)
    
    shifts = 0

    for crack in frame.raw_cracks:
        cords = crack.get_coordinates()
        for i in range(0,len(cords)-1):
            x, y = cords[i][0], cords[i][1]
            x1, y1 = cords[i+1][0], cords[i+1][1]
            cv2.line(blank_image, (x, y), (x1, y1), (0, 0, 255), thickness=2, lineType=4)
            
    for i in range(0,len(p1)-1):
        x, y = p1[i][0], p1[i][1]
        x1, y1 = p1[i+1][0], p1[i+1][1]
       # print(p.path[i])
        n = i/255
        n = math.floor(n)
        if ((n%2) == 0):
            color = i%255
        else:
            color = 255 - (i%255)
        #print(p.path[i][2],p.path[i+1][2])
        if (p1[i][2] and p1[i+1][2]):
            shifts += 1
            cv2.line(blank_image, (x, y), (x1, y1), (random.randint(100,255), random.randint(0,25),random.randint(0,255)), thickness=1, lineType=8)
            center = [math.floor(x+((x1-x)/2)), math.floor(y + ((y1-y)/2))]
            cv2.putText(blank_image, str(shifts),(center[0],center[1]),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0, 255, 0))
            #print("Shift")
        else:
            cv2.line(blank_image, (x, y), (x1, y1), (255, 255, 255), thickness=2, lineType=8)

        cv2.line(blank_image, (0, math.floor(offset)), (WIDTH, math.floor(offset)), (0, 255, 255), thickness=1, lineType=8)
        
    return blank_image

#Function for adding angles bounded to [0, 2*PI[
def angle_add(angle_1, angle_2):
    if(angle_1 + angle_2 >= 2*math.pi): return angle_1 + angle_2 - 2*math.pi
    elif(angle_1 + angle_2 < 0): return angle_1 + angle_2 + 2*math.pi
    else: return angle_1 + angle_2

#Function to transform a coordinate into world coordinates
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

def vision_pub(data_in, lock_in, event_receive, event_receive_ready):
        rospy.init_node('vision_publisher', anonymous=True)
        point_pub = rospy.Publisher('points', geo_msgs.PointStamped, queue_size = 10)
        transform_pub = rospy.Publisher('vo', nav_msgs.Odometry, queue_size = 50)

        # Transform listener for getting the ekf transform /vo_camera_frame -> /world_frame
        tf_buffer = tf2_ros.Buffer()#rospy.Time(100))
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        r = rospy.Rate(100) #100hz

        world_pose_x = 0
        world_pose_y = 0
        world_orientation = 0

        # Scalar from pixels to distances in camera frame
        PIXEL_SIZE = 1.6/WIDTH #In meters
        STANDARD_COVARIANCE = [0.05, 0.006, 0.01, 0.01, 0.01, 0.000009,
                            0.006, 0.05, 0.01, 0.01, 0.01, 0.01,
                            0.01, 0.01, 0.05, 0.01, 0.01, 0.01,
                            0.01, 0.01, 0.01, 0.09, 0.01, 0.01,
                            0.01, 0.01, 0.01, 0.01, 0.09, 0.01,
                            0.000009, 0.01, 0.01, 0.01, 0.01, 0.09]

        event_receive_ready.set()

        print('Thread 4 started (vision_pub)')
        while not rospy.is_shutdown():
            # Wait for event flag for new trajectory
            event_receive.wait()
            event_receive.clear()

            # Fetch trajectory
            lock_in.acquire()
            local_data = data_in.get_data()
            local_frame_time = data_in.get_frame_time()
            angle, traveled_x, traveled_y = data_in.get_image_offset()
            lock_in.release()

            event_receive_ready.set()
            #print('Thread 4 got data')

            # Split timestamp to secs and nsecs
            nsecs = local_frame_time % int(1000000000)
            secs = int((local_frame_time - nsecs)/1000000000)

            # Calculate world pose
            world_pose_x += traveled_x
            world_pose_y += traveled_y
            world_orientation = angle_add(world_orientation, angle)

            # Publish transform from camera
            tf_msg = nav_msgs.Odometry()
            tf_msg.header.stamp.secs = secs
            tf_msg.header.stamp.nsecs = nsecs
            tf_msg.header.frame_id = "world_frame"
            tf_msg.child_frame_id = "vo"

            # Add twist
            tf_msg.twist.twist.linear.x = traveled_x * PIXEL_SIZE
            tf_msg.twist.twist.linear.y = traveled_y * PIXEL_SIZE
            tf_msg.twist.twist.linear.z = 0.0
            tf_msg.twist.twist.angular.x = 0.0
            tf_msg.twist.twist.angular.y = 0.0
            tf_msg.twist.twist.angular.z = angle
            tf_msg.twist.covariance = STANDARD_COVARIANCE

            # Add pose
            tf_msg.pose.pose.position.x = world_pose_x * PIXEL_SIZE
            tf_msg.pose.pose.position.y = world_pose_y * PIXEL_SIZE
            tf_msg.pose.pose.position.z = 0.0
            quat = tf_convert.quaternion_from_euler(0, 0, world_orientation)
            tf_msg.pose.pose.orientation.x = quat[0]
            tf_msg.pose.pose.orientation.y = quat[1]
            tf_msg.pose.pose.orientation.z = quat[2]
            tf_msg.pose.pose.orientation.w = quat[3]
            tf_msg.pose.covariance = STANDARD_COVARIANCE

            transform_pub.publish(tf_msg)

            # Get transform from camera to world for current image
            try:
                transform_camera_to_world = tf_buffer.lookup_transform('world_frame', 'vo_camera_frame', rospy.Time(secs, nsecs))
            
                # Send each point in crack trajectory
                for path in local_data.path:
                    #geometry_msgs PointStamped Message
                    message = geo_msgs.PointStamped()
                    message.header.frame_id = "world_frame"
                    message.header.stamp.secs = secs
                    message.header.stamp.nsecs = nsecs
                    coords = transform_coordinates(path[0] * PIXEL_SIZE, path[1] * PIXEL_SIZE, transform_camera_to_world.transform)
                    message.point.x = coords[0]
                    message.point.y = coords[1]
                    message.point.z = path[2] # Used for sending the end of crack information

                    point_pub.publish(message)
                    r.sleep()

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exception:
                print(exception)
                # For first transform publish again to ensure transform is available for first point
                r.sleep()
                continue

#
def shutoffthread(process):
    import sys
    for p in range(len(process)):
        print("kill thread")
        p.terminate()
    sys.exit(1)


if __name__ == "__main__":
    BaseManager.register('DataTransfer', DataTransfer)

    # Locks
    thread_1_to_2_data_lock = Lock()
    thread_2_to_3_data_lock = Lock()
    thread_3_to_4_data_lock = Lock()

    img_raw_lock = Lock()
    img_seg_lock = Lock()
    path_lock = Lock()

    # Events
    thread_1_data_available = Event()  # Thread 1 has raw image ready
    thread_2_data_available = Event()  # Thread 2 has segmented image ready
    thread_3_data_available = Event()  # Thread 3 has path ready

    thread_2_ready_for_data = Event()  # Thread 2 is ready to receive raw image
    thread_3_ready_for_data = Event()  # Thread 3 is ready to receive segmented image
    thread_4_ready_for_data = Event()  # Thread 4 is ready to receive path

    img_raw_event = Event()     # Data available from thread 1
    img_seg_event = Event()     # Data available from thread 2
    transmit_event = Event()    # Data available from thread 3
    
    # Manager setup
    manager_raw_img = BaseManager()
    manager_seg_img = BaseManager()
    manager_path = BaseManager()
    
    manager_raw_img.start()
    manager_seg_img.start()
    manager_path.start()
    
    data_raw_img = manager_raw_img.DataTransfer()
    data_seg_img = manager_seg_img.DataTransfer()
    data_path = manager_path.DataTransfer()

    # Thread initialization
    t1 = Process(target=frames_from_files, args=(
        data_raw_img,
        img_raw_lock,
        thread_1_data_available,
        thread_2_ready_for_data
        ))
    t2 = Process(target=run_model, args=(
        data_raw_img, 
        data_seg_img,  
        img_raw_lock, 
        img_seg_lock, 
        thread_2_data_available,
        thread_3_ready_for_data,
        thread_1_data_available,
        thread_2_ready_for_data
        ))
    t3 = Process(target=path_planning, args=(
        data_seg_img, 
        data_path, 
        img_seg_lock,
        path_lock,
        thread_3_data_available,
        thread_4_ready_for_data,
        thread_2_data_available,
        thread_3_ready_for_data
        ))
    t4 = Process(target=vision_pub, args=(
        data_path, 
        path_lock, 
        thread_3_data_available,
        thread_4_ready_for_data
        ))

    # Start processes and join datatransmission
    processes = [t1,t2,t3,t4]
    
    # Shutdown extra threads when program is exiting
    atexit.register(shutoffthread, processes)

    for process in processes:
        process.start()

    for process in processes:
        process.join()   
