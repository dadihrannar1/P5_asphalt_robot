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


def frames_from_files(data_out, lock, event):
    path_string = current_path + '/test_images/realistic_split_'
    print('starting image load from: ' + path_string + '#.png')

    #start loading videos
    i = 0
    print('Thread 1 started (frames_from_files)')
    while True:
        if exists(path_string + str(i) + '.png'):
            frame_time = time.time_ns()
            frame = cv2.imread(path_string + str(i) + '.png')
            i = i + 1

            lock.acquire()
            data_out.set_data(frame)
            data_out.set_frame_time(frame_time)
            event.set()
            lock.release()

            time.sleep(5) #wait 5 seconds until noading next image
        else:
            exit('no more files to load')


##Function to run model from specifed path
# Run as a seperate thread for the program
def run_model(data_in, data_out, lock_in, lock_out, event_in, event_out):
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

    # Main thread loop
    print('Thread 2 started (run_model)')
    while(True):
        event_in.wait() # Wait for event flag from the image acquisition thread
        event_in.clear() # Clear event flag before processing starts (allows a new flag to arrive before processing finishes)

        # Lock data while it is being read by the thread
        lock_in.acquire()
        local_image = data_in.get_data()
        local_frame_time = data_in.get_frame_time()
        lock_in.release()
        
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
        
        # Send data to path planning thread
        lock_out.acquire()
        data_out.set_data(preds.cpu().numpy())
        data_out.set_frame_time(local_frame_time)
        data_out.set_image_offset(angle, traveled_x, traveled_y)

        # Set event flag for path planning thread
        event_out.set()
        lock_out.release()


def path_planning(data_in, data_out, lock_in, lock_out, event_in, event_out):
    img_old_seg = np.zeros(([2, 2]), dtype=np.uint8)
    old_frame = 0
    traveled = 0

    # Main thread loop
    print('Thread 3 started (path_planning)')
    while True:
        event_in.wait()    # Wait for new image
        event_in.clear()   # Clear event
        start_time = time.time()
        
        # Get data if lock is free
        lock_in.acquire()
        local_img = data_in.get_data().astype(np.uint8) # get data
        local_frame_time = data_in.get_frame_time()
        angle, traveled_x, traveled_y = data_in.get_image_offset()
        lock_in.release()
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
        
        # Set data if lock is free
        lock_out.acquire()
        data_out.set_data(frame1)
        data_out.set_frame_time(local_frame_time)
        data_out.set_image_offset(angle, traveled_x, traveled_y)
        event_out.set()
        lock_out.release()
        
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


"""
def transmit_trajectory(data_in, lock_in, event_in):
    # Main thread loop
    print('Thread 4 started (transmit_trajectory)')
    while True:
        # Wait for event flag for new trajectory
        event_in.wait()
        event_in.clear()

        # Fetch trajectory
        lock_in.acquire()
        local_data = data_in.get_data()
        lock_in.release()
        
        # Send each 
        for path in local_data.path:
            Data = {
                "Position": {
                     "X": path[0],
                     "Y": path[1]
                    },
                "Time": {"Detected": local_data.get_frame_time()},
                "Crack": {"DetectionIndex": path[2]} # Boolean (true when a crack starts or ends)
                }
            #print('Position: (X: ' + str(path[0]) + ', Y: ' + str(path[1]) + ')\nTime: ' + str(local_data.get_frame_time()) + ' Crack: ' + str(path[2]))
"""

#Function for adding angles bounded to [0, 2*PI[
def angle_add(angle_1, angle_2):
    if(angle_1 + angle_2 >= 2*math.pi): return angle_1 + angle_2 - 2*math.pi
    elif(angle_1 + angle_2 < 0): return angle_1 + angle_2 + 2*math.pi
    else: return angle_1 + angle_2

def vision_pub(data_in, lock_in, event_in):
        rospy.init_node('vision_publisher', anonymous=True)
        point_pub = rospy.Publisher('points', geo_msgs.PointStamped, queue_size = 10)
        transform_pub = rospy.Publisher('vo', nav_msgs.Odometry, queue_size = 50)

        r = rospy.Rate(100) #100hz

        world_pose_x = 0
        world_pose_y = 0
        world_orientation = 0

        STANDARD_COVARIANCE = [0.05, 0.006, 0.01, 0.01, 0.01, 0.000009,
                            0.006, 0.05, 0.01, 0.01, 0.01, 0.01,
                            0.01, 0.01, 0.05, 0.01, 0.01, 0.01,
                            0.01, 0.01, 0.01, 0.09, 0.01, 0.01,
                            0.01, 0.01, 0.01, 0.01, 0.09, 0.01,
                            0.000009, 0.01, 0.01, 0.01, 0.01, 0.09]

        print('Thread 4 started (vision_pub)')
        while not rospy.is_shutdown():
            # Wait for event flag for new trajectory
            event_in.wait()
            event_in.clear()

            # Fetch trajectory
            lock_in.acquire()
            local_data = data_in.get_data()
            local_frame_time = data_in.get_frame_time()
            angle, traveled_x, traveled_y = data_in.get_image_offset()
            lock_in.release()

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
            tf_msg.twist.twist.linear.x = traveled_x
            tf_msg.twist.twist.linear.y = traveled_y
            tf_msg.twist.twist.linear.z = 0.0
            tf_msg.twist.twist.angular.x = 0.0
            tf_msg.twist.twist.angular.y = 0.0
            tf_msg.twist.twist.angular.z = angle
            tf_msg.twist.covariance = STANDARD_COVARIANCE

            # Add pose
            tf_msg.pose.pose.position.x = world_pose_x
            tf_msg.pose.pose.position.y = world_pose_y
            tf_msg.pose.pose.position.z = 0.0
            quat = tf_convert.quaternion_from_euler(0, 0, world_orientation)
            tf_msg.pose.pose.orientation.x = quat[0]
            tf_msg.pose.pose.orientation.y = quat[1]
            tf_msg.pose.pose.orientation.z = quat[2]
            tf_msg.pose.pose.orientation.w = quat[3]
            tf_msg.pose.covariance = STANDARD_COVARIANCE

            transform_pub.publish(tf_msg)

            # Send each 
            for path in local_data.path:
                #geometry_msgs PointStamped Message
                message = geo_msgs.PointStamped()
                message.header.frame_id = "world_frame"
                message.header.stamp.secs = secs
                message.header.stamp.nsecs = nsecs
                message.point.x = path[0] 
                message.point.y = path[1] 
                message.point.z = path[2] # Used for sending the end of crack information

                point_pub.publish(message)
                r.sleep()


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
    img_raw_lock = Lock()
    img_seg_lock = Lock()
    path_lock = Lock()

    # Events
    img_raw_event = Event()     # Start thread 2
    img_seg_event = Event()     # Start thread 3
    transmit_event = Event()    # Start thread 4
    
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
        img_raw_event
        ))
    t2 = Process(target=run_model, args=(
        data_raw_img, 
        data_seg_img,  
        img_raw_lock, 
        img_seg_lock, 
        img_raw_event,
        img_seg_event,))
    t3 = Process(target=path_planning, args=(
        data_seg_img, 
        data_path, 
        img_seg_lock,
        path_lock,
        img_seg_event,
        transmit_event
        ))
    t4 = Process(target=vision_pub, args=(
        data_path, 
        path_lock, 
        transmit_event
        ))

    # Start processes and join datatransmission
    processes = [t1,t2,t3,t4]
    
    # Shutdown extra threads when program is exiting
    atexit.register(shutoffthread, processes)

    for process in processes:
        process.start()

    for process in processes:
        process.join()   


          



    # Thread initialization



    


"""

if __name__ == "__main__":
    BaseManager.register('DataTransfer', DataTransfer)

    # Locks
    img_raw_lock = Lock()
    img_seg_lock = Lock()
    path_lock = Lock()

    # Events
    img_raw_event = Event()     # Start thread 2
    img_seg_event = Event()     # Start thread 3
    transmit_event = Event()    # Start thread 4
    
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
        img_raw_event
        ))
    t2 = Process(target=run_model, args=(
        data_raw_img, 
        data_seg_img,  
        img_raw_lock, 
        img_seg_lock, 
        img_raw_event,
        img_seg_event,))
    t3 = Process(target=path_planning, args=(
        data_seg_img, 
        data_path, 
        img_seg_lock,
        path_lock,
        img_seg_event,
        transmit_event
        ))
    t4 = Process(target=transmit_trajectory, args=(
        data_path, 
        path_lock, 
        transmit_event
        ))

    # Start processes and join datatransmission
    processes = [t1,t2,t3,t4]

    for process in processes:
        process.start()

    for process in processes:
        process.join()
"""