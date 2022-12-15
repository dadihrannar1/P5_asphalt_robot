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

    print('Started processing images')
    for i, filename in enumerate(filename[start_image:end_image]):
        frame = cv2.imread(image_path + "/" + filename)

        # Undistort image, followed by rotation and cropping
        frame_corrected = cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)
        frame_rotated = cv2.rotate(frame_corrected, cv2.ROTATE_90_CLOCKWISE)
        # Crop image to remove trailer edges
        frame_cropped = frame_rotated[100:1720, 100:980]

        # Determine odometry from images
        if img_raw_old.any():
            new_image = cv2.resize(frame_cropped,(WIDTH,HEIGHT),interpolation=cv2.INTER_AREA)
            new_image = cv2.cvtColor(new_image,cv2.COLOR_BGR2GRAY)
            
            angle, traveled_x, traveled_y = image_aligner.visual_odometry(img_raw_old, new_image)

        # Save image for odometry calculation
        img_raw_old = np.copy(frame_cropped)
        img_raw_old = cv2.resize(img_raw_old,(WIDTH,HEIGHT),interpolation=cv2.INTER_AREA)
        img_raw_old = cv2.cvtColor(img_raw_old,cv2.COLOR_BGR2GRAY)

        # Use UNET to detect cracks
        augmentented = detect_transform(image=frame_cropped)
        data = augmentented["image"].to(device=DEVICE)
        data = torch.unsqueeze(data, 0)
        output = torch.sigmoid(model(data))
        output = torch.squeeze(output)
        preds = (output > 0.5).float()

        #
        sorted_cracks = process_image(preds.cpu().numpy())
        frame1 = Frame()
        for crack in sorted_cracks:
            frame1.add_crack(Crack(crack))

        # If not first frame, ensure all cracks which have already been marked for repair are not marked again
        if not old_frame == 0:
            map_cracks(old_frame, frame1, int(traveled_y))

        # Determine best path through cracks
        frame1.find_path()
        old_frame = copy.copy(frame1)

        print(f"vision_node: {filename} saved")
        filenames.append(filename)
        paths.append(frame1.path)
        timestamps.append(arduino_timestamp[i])
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
