import json
import cv2
import pickle
import numpy as np

# Set to true when debugging
debug = True


# Function for displaying images -> not important
def resize_image(image, image_name, procent):
    [height, width] = [image.shape[0],image.shape[1]]
    [height, width] = [procent*height, procent*width]
    cv2.namedWindow(image_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(image_name, int(width), int(height))
    cv2.imshow(image_name, image)


# Function for extracting an estimation of the transform between images
def get_transform(encoder1, encoder2):
    enc_rads = (encoder1 / 100, encoder2 / 100)
    theta_w = (0.38 * np.pi) / 1.466 * (enc_rads[0] - enc_rads[1])
    x_tran = (0.38*np.pi)/2*(sum(enc_rads))#*theta_w
    y_tran = (0.38 * np.pi) / 2 * (sum(enc_rads))#*theta_w
    transform = [x_tran, y_tran]
    return transform


# Function for stitching images together based off the encoder values
def stitch_images(images, offsets):
    resized_images = []
    for (image,offset) in zip(images, offsets):
        if offset == "a":
            resized_images.append(image)
            continue
        w = image.shape[0]
        h = image.shape[1]
        top_left_corner = (int(w/2-offset/2), 0)
        bottom_right_corner = (int(w/2+offset/2), h)
        resized_images.append(image[top_left_corner[1]:bottom_right_corner[1], top_left_corner[0]:bottom_right_corner[0]])
    result = cv2.hconcat(resized_images)
    return result


# Set desired size of image
desired_w, desired_h = (int(16384/1.5),int(4096/1.5))
# Sizes of variables
pixelSize = 0.8853  # mm/pixel
tickSize = 11.9381  # encoder size
tickPixel = tickSize /pixelSize  # encoder ticks to pixel size
print(tickPixel)

# define path to find the json and images
path = r"C:\Users\Muku\OneDrive - Aalborg Universitet\P5 - Crack Robot\Images_lang2"

# calibration values for the camera
with open(r"calib_matrix.pkl", 'rb') as file:
    map_x = pickle.load(file)
    map_y = pickle.load(file)

# Open the json file and extract data
with open(f"{path}/image_details.json") as json_file:
    data = json.load(json_file)
    filename = data[0]
    timer = [int(s) for s in data[1]]  # Converting string to int
    encoder1 = [int(s) for s in data[2]]
    encoder2 = [int(s) for s in data[3]]

# define a stitcher
n_images = 30
starting_image = 1030
images = []
transforms = []
for j in range(starting_image,starting_image+n_images):
    img_name = filename[j].split("/")[-1]
    img = cv2.imread(f"{path}/{img_name}")
    frame_corrected = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
    top_left_corner = (150, 100)
    bottom_right_corner = (frame_corrected.shape[1] - 150, frame_corrected.shape[0] - 100)
    images.append(frame_corrected[top_left_corner[1]:bottom_right_corner[1], top_left_corner[0]:bottom_right_corner[0]])
    #resize_image(frame_corrected[top_left_corner[1]:bottom_right_corner[1], top_left_corner[0]:bottom_right_corner[0]], f"original{j}", 0.4)
    transforms.append(encoder2[j+1]*tickPixel)
transforms.pop(-1)
transforms.append("a")
14
result = stitch_images(images, transforms)
h, w = result.shape[:2]
# increase the size so it fits in webots
top = (desired_h - h) // 2
bottom = desired_h - h - top
left = (desired_w - w) // 2
right = desired_w - w - left
# Add a black border so that it has the right size
bordered_image = cv2.copyMakeBorder(result, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(0,0,0))


# Write out the results and save the image
cv2.imwrite(f'sitched_{starting_image+1}-{starting_image+n_images}_image.png', bordered_image)
realworldsize = [bordered_image.shape[0]*pixelSize,bordered_image.shape[1]*pixelSize]
print(bordered_image.shape)
print(realworldsize)

# Debugging
if debug:
    image_gray = cv2.cvtColor(bordered_image, cv2.COLOR_BGR2GRAY)
    template_gray = cv2.cvtColor(images[0], cv2.COLOR_BGR2GRAY)
    template_height, template_width = template_gray.shape[:2]

    #for i, image in enumerate(images):
    #    resize_image(image, f'original{i}', 0.2)
    #resize_image(bordered_image,'resized', 0.2)
    resize_image(result, "stitched", 0.3)
    cv2.waitKey(0)
