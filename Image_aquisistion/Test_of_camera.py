# import the opencv library
import cv2 as cv
import time
import json
import numpy as np  

# define a video capture object
vid = cv.VideoCapture(0)
frequensy_of_images= 1 #seconds between image is taken
num_image=0
image = []
timer = []


while(True):
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    if cv.waitKey(1) & 0xFF == ord('s'):
       timer.append(time.time())
       filename="Image_aquisistion/Images/saved_image_%s.png" % num_image
       image.append(filename)
       cv.imwrite(filename, frame)
       num_image = num_image+1
       print(num_image)

       

    # Display the resulting frame
    cv.imshow('frame', frame)
      
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
full_list = [image, timer]
dump = json.dumps(full_list)
with open("Image_aquisistion/image_details.json", "w") as outfile:
    outfile.write(dump)
# Destroy all the windows
cv.destroyAllWindows()