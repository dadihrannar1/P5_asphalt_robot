import cv2 as cv
from os.path import exists
from os import mkdir, listdir
import numpy as np

def resizenation(image, percentage):
    scale_percent = percentage  # percent of original size #this is just to make the program faster
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    return cv.resize(image, dim, interpolation=cv.INTER_AREA)
#folder where the training videos are
folder = 'Training/Videos'
#name of the injury
files_name = "Cracking"

#name of the folder where the training images will land
image_folder_name = "Training/image"

#creates a folder for all the training data
if not exists("./%s" % image_folder_name):
    mkdir("./%s" % image_folder_name)
#path of the folder of your choice
path = f"{folder}"
listofvids = listdir(path)

# loops through all the videos and shows them to you, if s is pressed image is saved
for j, vid in enumerate(listofvids):
    # Puts out a string type of the name of the path to the three types of video.
    spit = vid.split(".")
    if spit[1] == "MP4":
        bgrvideo = f"{folder}/{listofvids[j]}"
        types = spit[0]
        filenum = 0

        # If the path to the three videos exists create a sub folder of the damage type and then subfolders for
        # the three video types
        if exists(bgrvideo):
            #reads the frames of the three videos
            bgrcap = cv.VideoCapture(bgrvideo)
            while bgrcap.isOpened():
                ret, frame = bgrcap.read()
                if not ret:
                    print("next video")
                    break
                resized = resizenation(frame, 20)
                cv.imshow("BGR image", resized)
                key = cv.waitKey(10)
                if key == ord("q"):
                    break
                if key == ord("s"):
                    filenum = str(filenum)
                    filenum = filenum.zfill(3)
                    bgrfilename = f"{image_folder_name}/{filenum}_{files_name}.png"
                    cv.imwrite(bgrfilename, frame)
                    filenum = int(filenum)
                    filenum = filenum+1