import glob                                                           
import cv2 
pngs = glob.glob('./src/P5_asphalt_robot/new_controller/Long_Crack_for_testing/time_to_jpg/*.png')
print(pngs)
for j in pngs:
    img = cv2.imread(j)
    cv2.imwrite(j[:-3] + 'jpg', img)