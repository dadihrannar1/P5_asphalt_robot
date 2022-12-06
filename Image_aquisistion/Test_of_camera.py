import os
import cv2 as cv
import time
import json
import serial
# opens connections to arduino
arduino = serial.Serial(port='COM8', baudrate=57600, timeout=.1)
time.sleep(0.1)
arduino.flushInput()
# number written to the arduino to synchronise python and the arduino, if it does'nt try putting a 1 inside the "".
sync = "1"


# reformating the serial read from the arduino
def conversion():
    data = str(arduino.readlines())
    print(data)
    data=data.split("'") 
    data = data[1].split("$")
    delta1 = data[0]
    delta2 = data[1]
    time_stamp = data[2]
    time_stamp = time_stamp.split("\\")
    time_stamp = time_stamp[0]
    return delta1, delta2, time_stamp


tik = time.time()

# define a video capture object
vid = cv.VideoCapture(2)

frequensy_of_images= 1 #seconds between image is taken
num_image = 0 # variable that iterates for image taken

# Storing image filename, the time the image is takne, and the delta value of the encoder
image = [] 
timer = []
encoder1 = []
encoder2 = []

save_path = "Image_aquisistion/Images"
if not os.path.exists(os.getcwd() + "/" + save_path):
    os.makedirs(os.getcwd() + "/" + save_path)

while(True):
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    # Saves the frame as a png file and grabs data from the arduino
    if time.time()-tik >= frequensy_of_images:
        arduino.write(bytes(sync, 'utf-8'))
        delta1, delta2, time_stamp = conversion()
        timer.append(time_stamp)
        encoder1.append(delta1)
        encoder2.append(delta2) 
        filename=f"{save_path}/saved_image_{num_image}.png"
        image.append(filename)
        cv.imwrite(filename, frame)
        num_image = num_image+1
        tik = time.time()

    # Display the resulting frame
    cv.imshow('frame', frame)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
full_list = [image, timer, encoder1,encoder2]
dump = json.dumps(full_list)
with open("Image_aquisistion/Images/image_details.json", "w") as outfile:
    outfile.write(dump)

# Destroy all the windows
cv.destroyAllWindows()
