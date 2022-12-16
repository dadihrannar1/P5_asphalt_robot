#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
import torch
import albumentations
from albumentations.pytorch.transforms import ToTensorV2
import copy
from pathlib import Path
from model_utils import load_model
import image_aligner
from trajectory_planning import Crack, Frame, map_cracks, process_image
import atexit
import pickle
import json
import math
import random
from vision.srv import Display_input, Display_inputRequest

# Camera source (0 for webcam)
capture_src = 0

# Path to model
current_path = Path(__file__).parent
model_name = 'TrainedModel.pth.tar'

# Processing device for running model
DEVICE = 'cpu'

# Parameters for image size
HEIGHT = int(1080/2)
WIDTH = int(1920/2)

def shutoff_savedata(data, file_path):
    import sys
    print(f"On exit saved to {file_path}")
    with open(file_path, 'wb') as f:
        pickle.dump(data, f)
    sys.exit()

#Visualise cracks for debugging
def visualize(frame: Frame, p1, offset):
    blank_image = np.zeros((WIDTH,HEIGHT,3), np.uint8)
    
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

        #cv2.line(blank_image, (0, math.floor(offset)), (WIDTH, math.floor(offset)), (0, 255, 255), thickness=1, lineType=8)
        
    return blank_image

if __name__ == "__main__":
    rospy.init_node('vision_frame', anonymous=True)

    # Load trained model ffrom specifed path
    model = load_model("/media/sf_shared_files" + '/' + model_name)

    # Transforms
    detect_transform = albumentations.Compose([
        albumentations.Resize(height=HEIGHT, width=WIDTH),
        albumentations.Normalize(
            mean=[0.0, 0.0, 0.0],
            std=[1.0, 1.0, 1.0],
            max_pixel_value=255.0,
        ),
        ToTensorV2()])

    # Load camera intrinsics
    with open(str(current_path) + '/calib_matrix.pkl', 'rb') as file:
        map_x = pickle.load(file)
        map_y = pickle.load(file)
    
    # Get image path from roslaunch
    image_path = rospy.get_param("~Image_path")
    start_image = rospy.get_param("~Start_image")
    end_image = rospy.get_param("~End_image")
    if(type(end_image) != int):
        exit("Vision_node: WRONG LAUNCH INPUT TYPE")
    if(end_image<start_image):
        exit("Vision_node: Invalid launch input end<start")

    
    # Start the image stitcher with the same path as the images loaded in the vision node
    #request = Display_inputRequest()
    #request.path = image_path # Path must contain json file and images
    #request.start_image = start_image # first image number
    #request.amount_of_images = amount_of_images # number of images (max ~60)

    #print("Thread 1: Sending images to the simulation\n")
    #image_stitch = rospy.ServiceProxy('/input_display', Display_input)
    #framePublisher = rospy.Publisher('/frame_publisher', Bool, queue_size=5)
    #image_stitch.call(request)

    # Set various comparion values for first runtrhough
    img_raw_old = np.zeros(([2, 2]), dtype=np.uint8)
    old_frame = 0
    traveled_x = 0
    traveled_y = 0
    angle = 0

    # pickle file path for output
    file_path = str(current_path) + '/vision_out.pkl'
    pkl_data = []
    filenames = []
    paths = []
    timestamps = []
    offsets = []

    #atexit.register(shutoff_savedata, file_path, pkl_data)

    # Read JSON file
    with open(f"{image_path}/image_details.json") as json_file:
        data = json.load(json_file)
        filename = [img.split("/")[-1] for img in data[0]]
        arduino_timestamp = [int(s) for s in data[1]]  # Converting string to int
        encoder1 = [int(s) for s in data[2]]
        encoder2 = [int(s) for s in data[3]]

    arduino_time_differences = []
    arduino_time_differences.append(arduino_timestamp[0])
    for i in range(1,len(arduino_timestamp)):
        arduino_time_differences.append(arduino_timestamp[i] - arduino_timestamp[i-1])

    print('Started processing images')
    for filename, arduino_time_difference in zip(filename[start_image:end_image], arduino_time_differences[start_image:end_image]):
        frame = cv2.imread(image_path + "/" + filename)

        # Undistort image, followed by rotation and cropping
        frame_corrected = cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)
        frame_cropped = frame_corrected[100:980, 100:1720]
        frame_rotated = cv2.rotate(frame_cropped, cv2.ROTATE_90_CLOCKWISE)

        # DEBUG Display resized image
        #height, width = frame_rotated.shape[:2]
        #frame_show = cv2.resize(frame_rotated, (width//2, height//2))
        #cv2.imshow('image', frame_show)
        #cv2.waitKey(10)


        # Determine odometry from images
        if img_raw_old.any():
            new_image = cv2.cvtColor(frame_rotated,cv2.COLOR_BGR2GRAY)
            
            angle, traveled_x, traveled_y = image_aligner.visual_odometry(img_raw_old, new_image, 10000, keep_best_ratio=0.1)
            #print(f"{angle*180/math.pi}\t{traveled_x}\t{traveled_y}")

        # Save image for odometry calculation
        img_raw_old = np.copy(frame_rotated)
        img_raw_old = cv2.cvtColor(img_raw_old,cv2.COLOR_BGR2GRAY)

        # Use UNET to detect cracks
        augmentented = detect_transform(image=frame_corrected)
        data = augmentented["image"].to(device=DEVICE)
        data = torch.unsqueeze(data, 0)
        output = torch.sigmoid(model(data))
        output = torch.squeeze(output)
        preds = (output > 0.5).float()
        predictions_cropped = preds.cpu().numpy()[100:980, 100:1720]    # Crop image to remove trailer edges
        predictions_rotated = cv2.rotate(predictions_cropped, cv2.ROTATE_90_CLOCKWISE)  # Rotate

        #DEBUG show predictions
        #cv2.imshow('imageprediction', predictions_rotated)
        #cv2.waitKey(10)

        #
        sorted_cracks = process_image(predictions_rotated)

        frame1 = Frame()
        for crack in sorted_cracks:
            frame1.add_crack(Crack(crack))

        # If not first frame, ensure all cracks which have already been marked for repair are not marked again
        if not old_frame == 0:
            map_cracks(old_frame, frame1, int(traveled_y))

        # Determine best path through cracks
        frame1.find_path()
        old_frame = copy.copy(frame1)

        frame0Vis = visualize(frame1, frame1.path, traveled_x)
        cv2.imshow("crack visualisation", frame0Vis)
        cv2.waitKey(10)

        print(f"vision_node: {filename} saved")
        filenames.append(filename)
        paths.append(frame1.path)
        timestamps.append(arduino_time_difference)
        offsets.append((angle, traveled_x, traveled_y))
        pkl_data = [filenames, paths, timestamps, offsets]

    with open(f'{image_path}/vision_output.pkl', 'wb') as f:
        # Dumping files in order 
        # 1: Filenames
        # 2: Paths 
        # 3: Timestamps
        # 4: Offsets
        pickle.dump(pkl_data[0], f)
        pickle.dump(pkl_data[1], f)
        pickle.dump(pkl_data[2], f)
        pickle.dump(pkl_data[3], f)
    print(f'Saved to {image_path}')
