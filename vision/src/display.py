#!/usr/bin/env python3
import json
import cv2
import pickle
import time
from pathlib import Path
import numpy as np


# Import the srv and msg messages
from vision.srv import Display_input, Display_inputRequest
from vision.srv import Draw_workspace
from webots_ros.srv import display_image_load, display_image_loadRequest
from webots_ros.srv import display_image_paste, display_image_pasteRequest
from webots_ros.srv import display_draw_oval, display_draw_ovalRequest
from webots_ros.srv import set_float, set_floatRequest
from webots_ros.srv import set_int
from webots_ros.msg import Float64Stamped
from std_msgs.msg import Bool
from std_msgs.msg import Float32

# Import the rospy library
import rospy


# Function for displaying images -> not important
def resize_image(image, image_name, procent):
    [height, width] = [image.shape[0],image.shape[1]]
    [height, width] = [procent*height, procent*width]
    cv2.namedWindow(image_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(image_name, int(width), int(height))
    cv2.imshow(image_name, image)


# Function for stitching images together based off the encoder values
def stitch_images(images, offsets):
    resized_images = []
    for (i, (image,offset)) in enumerate(zip(images, offsets)):
        w = image.shape[0]
        h = image.shape[1]
        top_left_corner = (int(w/2-offset/2), 0)
        bottom_right_corner = (int(w/2+offset/2), h)
        if i == len(images)-1: # last image should load the entire image
            bottom_right_corner = (w, h)
        elif i == 0: # The first image should only crop half of the image
            top_left_corner = (0, 0)
        resized_images.append(image[top_left_corner[1]:bottom_right_corner[1], top_left_corner[0]:bottom_right_corner[0]])
    result = cv2.hconcat(resized_images)
    return result


class DisplayService(object):
    # Set desired size of image
    desired_w, desired_h = (int(16384/1.5),int(4096/1.5))
    # Sizes of variables
    pixelSize = 0.8853  # mm/pixel
    tickSize = 380*np.pi/100 # mm - encoder size
    tickPixel = tickSize /pixelSize  # encoder ticks to pixel size
    image_path = ""

    # Callback function for retrieving the encoder data
    def encoder_callback(self, msg):
        self.encpos = msg.data

        if self.encpos >= self.image_edge:
            # Set up the speed so we can move back to the start
            self.vel_motor_client.call(10000)

            # make the motor go back to start 
            self.pos_motor_client.call(self.start_point)

            # Set the velocity back
            self.vel_motor_client.call(self.curr_vel)

    # Service set to start of reset the simulation
    def setState(self, state):
        if state == False:
            # Set up the speed so we can move back to the start
            self.vel_motor_client.call(10000)

            # make the motor go back to start 
            self.pos_motor_client.call(self.start_point)

            # Set the velocity back
            self.vel_motor_client.call(self.curr_vel)
        else:
            # make the motor go to an end 
            self.pos_motor_client.call(20.0) # Units are in meters
        return True


    # Constructer
    def __init__(self):
        self.model_name = "fivebarTrailer"
        self.image_edge = 1.5
        self.start_point = -1.5
        self.curr_vel = 0.0

        # Set services to advertise
        self.displayService = rospy.Service('input_display', Display_input, self.setDisplay)
        self.speedService = rospy.Service('set_display_velocity', set_float, self.setSpeed)
        self.drawingService = rospy.Service('draw_in_workspace', Draw_workspace, self.drawInWorkspace)



        # Set up publishers
        self.state_publisher = rospy.Publisher('display_state', Bool, queue_size=10)
        self.state_publisher.publish(False)
        self.vel_publisher = rospy.Publisher('velocity', Float32, queue_size=10)

        # Set up the display services
        self.display_image_load_client = rospy.ServiceProxy(f"{self.model_name}/CrackDisplay1/image_load", display_image_load)
        self.display_image_paste_client  = rospy.ServiceProxy(f"{self.model_name}/CrackDisplay1/image_paste", display_image_paste)
        self.display_image_draw_client = rospy.ServiceProxy(f"{self.model_name}/CrackDisplay1/fill_oval", display_draw_oval)
        self.display_image_color_client = rospy.ServiceProxy(f"{self.model_name}/CrackDisplay1/set_color", set_int)

        # start the encoder and set a subscriber
        encService = rospy.ServiceProxy(f"{self.model_name}/position_sensor1/enable", set_int)
        encService.call(4) # enable position sensor -> 32 is the rate
        rospy.Subscriber(f"{self.model_name}/position_sensor1/value", Float64Stamped, self.encoder_callback)

        # Motor clients
        self.vel_motor_client = rospy.ServiceProxy(f"{self.model_name}/Display_motor1/set_velocity", set_float)
        self.pos_motor_client = rospy.ServiceProxy(f"{self.model_name}/Display_motor1/set_position", set_float)

    def setDisplay(self, request):
        # Set indication that the display is busy
        self.state_publisher.publish(False)

        path = request.path
        n_images = request.amount_of_images
        starting_image = request.start_image

        parent_path = str(Path(__file__).parent)
        # calibration values for the camera
        with open(f"{parent_path}/calib_matrix.pkl", 'rb') as file:
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
        images = []
        transforms = []
        # Go through the requested amount of images
        for j in range(starting_image,starting_image+n_images):
            img_name = filename[j].split("/")[-1]
            img = cv2.imread(f"{path}/{img_name}")
            frame_corrected = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR) # Remove the distortion
            top_left_corner = (100, 100)
            bottom_right_corner = (frame_corrected.shape[1] - 100, frame_corrected.shape[0] - 100) # remove edges
            images.append(frame_corrected[top_left_corner[1]:bottom_right_corner[1], top_left_corner[0]:bottom_right_corner[0]])
            transforms.append(encoder1[j+1]*self.tickPixel) # save the encoder position
        #transforms.pop(-1) # Remove the last encoder position since we don't are about the image after it
        #transforms.append("a")

        # Switch the images together
        result = stitch_images(images, transforms)
        h, w = result.shape[:2]
        
        # Picture distance from local/origo
        distance_from_center = 1530.5/1000 # Distance found in solidworks
        distance_from_origo = distance_from_center-(1920*self.pixelSize)/2000

        self.start_point = -(w*self.pixelSize)/2000-distance_from_origo
        self.image_edge = (w*self.pixelSize)/2000
        # increase the size so it fits in webots
        top = (self.desired_h - h) // 2
        bottom = self.desired_h - h - top
        left = (self.desired_w - w) // 2
        right = self.desired_w - w - left
        # Add a black border so that it has the right size
        
        bordered_image = cv2.copyMakeBorder(result, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(0,0,0))

        # Write out the results and save the image
        image_path = f'{parent_path}/sitched_{starting_image+1}-{starting_image+n_images}_image.jpg'
        cv2.imwrite(image_path, bordered_image)

        # import the image into webots
        response = self.display_image_load_client.call(image_path)
        request = display_image_pasteRequest()
        request.ir = response.ir
        self.display_image_paste_client.call(request)

        # Set up the speed so we can move back to the start
        self.vel_motor_client.call(10000)
        time.sleep(0.1)
        # make the motor go back to start 
        self.pos_motor_client.call(self.start_point)
        # Set the motor speed back to zero
        time.sleep(0.5)
        
        self.vel_motor_client.call(0)

        
        
        # Calculate starting speed
        start_speed = ((encoder1[starting_image+1]*self.tickSize)/(timer[starting_image+1]-timer[starting_image])) # m/s
        self.curr_vel = start_speed
        self.curr_vel = 0.277778/5
        self.vel_motor_client.call(self.curr_vel)
        self.vel_publisher.publish(self.curr_vel)
        self.pos_motor_client.call(20)

        # Tell the rosserver display is ready to start
        self.state_publisher.publish(True)
        self.setState(True)
        return 1

    # A service for setting the speed of the display
    def setSpeed(self, msg):
        self.curr_vel = msg.value # motor speed is in m/s
        self.vel_motor_client.call(self.curr_vel)
        self.vel_publisher.publish(self.curr_vel)
        return 1
    
    # a service that draws in the workspace according to the manipulator position
    def drawInWorkspace(self, msg):
        #print(f'Display recieved positions: {msg.x}, {msg.y}, {msg.radius}')
        recieved_x = msg.x/self.pixelSize
        recieved_y = msg.y/self.pixelSize
        recieved_r = msg.radius

        # Find the position in world coordinates
        L0 = 176 # This is the robot distance between motors

        zero_pos = (self.desired_h/2+(L0/self.pixelSize/2), (self.encpos*1000)/self.pixelSize+self.desired_w/2)
        cy,cx = (zero_pos[0]-recieved_x, zero_pos[1]-recieved_y)

        # Draw image
        a, b = (recieved_r, recieved_r)
        self.display_image_draw_client.call(int(cx), int(cy), a, b)
        time.sleep(0.01)
        return True
    


##### Main loop #####

rospy.init_node('webots_display')
# Wait for the simulation controller.
rospy.wait_for_service("/fivebarTrailer/robot/time_step")
time.sleep(0.1)

display = DisplayService()

# Set the drawing color to red
red_color = 8914952
display.display_image_color_client.call(red_color)
time.sleep(0.1)

rospy.Rate(1000)

rospy.spin()


