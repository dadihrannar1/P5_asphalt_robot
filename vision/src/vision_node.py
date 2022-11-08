#!/usr/bin/env python3

import numpy as np
import cv2
import time
import math
import random
import rospy
import torch
import albumentations
from albumentations.pytorch.transforms import ToTensorV2
from multiprocessing import Process, Lock, Event
from multiprocessing.managers import BaseManager
import copy
from os.path import exists
from pathlib import Path
from model_utils import load_model
from image_aligner import image_aligner_cpu
from trajectory_planning import Crack, Frame, map_cracks, process_image


from vision.msg import vision_out


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
        self.offset = 0
        self.frame_time = 0

    def set_data(self,value):
        self.data = value

    def get_data(self):
        return self.data

    def set_image_offset(self, value):
        self.offset = value

    def get_image_offset(self):
        return self.offset

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
            frame_time = time.time()
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
            frame_time = time.time()
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
    traveled = 0

    # Main thread loop
    print('Thread 2 started (run_model)')
    while(True):
        event_in.wait() # Wait for event flag from the image acquisition thread
        event_in.clear() # Clear event flag before processing starts (allows a new flag to arrive before processing finishes)

        # Lock data while it is being read by the thread
        lock_in.acquire()
        local_image = data_in.get_data()
        local_frame_time = data_in.get_frame_time() #TODO this seems unused why is it here
        lock_in.release()
        
        # 
        if img_raw_old.any():
            new_image = cv2.resize(local_image,(WIDTH,HEIGHT),interpolation=cv2.INTER_AREA)
            new_image = cv2.cvtColor(new_image,cv2.COLOR_BGR2GRAY)
            traveled, overlapHeight = image_aligner_cpu(img_raw_old, new_image)

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
        # data_out.set_frame_time(local_frameTime)
        data_out.set_image_offset(traveled)

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
        offset = data_in.get_image_offset()
        lock_in.release()
        sorted_cracks = process_image(local_img)
        img_old_seg = local_img
        frame1 = Frame()    
        frame1.set_frame_time(local_frame_time)

        for crack in sorted_cracks:
            frame1.add_crack(Crack(crack))

        if not old_frame == 0:
            map_cracks(old_frame,frame1,offset)
        
        frame1.find_path()
       
        old_frame = copy.copy(frame1)
        # from path_planning.utils import map_cracks
        # if not old_frame == 0:
        #     map_cracks()
        
        # Set data if lock is free
        lock_out.acquire()
        data_out.set_data(frame1)
        event_out.set()
        lock_out.release()
        
        frame0Vis = visualize(frame1, frame1.path,320*0.75)
        cv2.imshow("crack visualisation", frame0Vis)
        cv2.waitKey(10)


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

def talker(data_in, lock_in, event_in):
        print("x")
        pub = rospy.Publisher('custom_chatter', vision_out, queue_size = 10)
        rospy.init_node('custom_talker', anonymous=True)
        r = rospy.Rate(10) #10hz
        while True:
            print("B")
            #event_in.wait()
            #event_in.clear()
            print("y")

            # Fetch trajectory
            lock_in.acquire()
            local_data = data_in.get_data()
            lock_in.release()
            msg = vision_out()
            msg.x = local_data.path[0]
            #msg.y = local_data.path[1]
            #msg.crack = local_data.path[2]
            #msg.time = local_data.get_frame_time()

            while not rospy.is_shutdown():
                rospy.loginfo(msg)
                pub.publish(msg)
                r.sleep()


if __name__ == "__main__":
    
    try:
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
        
        talker(data_path, path_lock, transmit_event)

        t4 = Process(target=transmit_trajectory, args=(
        data_path, 
        path_lock, 
        transmit_event
        
        ))

        
    except rospy.ROSInterruptException: pass          



    # Thread initialization



    

    # Start processes and join datatransmission
    processes = [t1,t2,t3,t4]

    for process in processes:
        process.start()

    for process in processes:
        process.join()

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