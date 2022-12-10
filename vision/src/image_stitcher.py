import json
import cv2
import pickle

debug = True

desired_w, desired_h = (10000,2000)
path = r"C:\Users\Muku\OneDrive - Aalborg Universitet\P5 - Crack Robot\Images_lang2"

def resize_image(image, image_name, procent):
    [height, width] = [image.shape[0],image.shape[1]]
    [height, width] = [procent*height, procent*width]
    cv2.namedWindow(image_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(image_name, int(width), int(height))
    cv2.imshow(image_name, image)

# calibration values for the camera
with open(r"calib_matrix.pkl", 'rb') as file:
    map_x = pickle.load(file)
    map_y = pickle.load(file)

with open(f"{path}/image_details.json") as json_file:
    data = json.load(json_file)
    filename = data[0]
    timer = data[1]
    encoder1 = data[2]
    encoder2 = data[3]

pixelSize = 0.9712
tickSize = 277.8 / 100
tickPixel = pixelSize / tickSize

# define a stitcher


for i in range(1030,1050,20):
    images = []
    stitcher = cv2.Stitcher.create(mode=cv2.STITCHER_SCANS)
    for j in range(i,i+20):
        img_name = filename[j].split("/")[-1]
        img = cv2.imread(f"{path}/{img_name}")
        frame_corrected = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
        images.append(frame_corrected[100:1720, 100:980])

    pixelOffset = int(int(encoder1[i+1])/tickPixel)+1
    stitcher.setPanoConfidenceThresh(1)
    result = stitcher.stitch(images)

    h, w = result[1].shape[:2]
    # increase the size so it fits in webots
    top = (desired_h - h) // 2
    bottom = desired_h - h - top
    left = (desired_w - w) // 2
    right = desired_w - w - left
    bordered_image = cv2.copyMakeBorder(result[1], top, bottom, left, right, cv2.BORDER_CONSTANT, value=(0,0,0))
    if debug:
        resize_image(bordered_image,'resized', 0.2)
        resize_image(result[1], "stitched", 0.2)
        cv2.waitKey(0)
    cv2.imwrite(f'sitched_{i+1}-{i+21}_image.png', bordered_image[1])
    realworldsize = [bordered_image.shape[0]*pixelSize,bordered_image.shape[1]*pixelSize]
    print(realworldsize)