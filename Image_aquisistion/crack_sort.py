def sort(image_folder):
    import cv2 as cv
    import os 

    os.mkdir("%s/cracks" % image_folder)
    os.mkdir("%s/non_cracks" % image_folder)
    for images in os.listdir(image_folder):
        if images !="cracks" and images != "non_cracks":
            filename="%s/%s" % (image_folder,images)
            img = cv.imread(filename)
            cv.imshow("hej",img)
            key = cv.waitKey(0)
            if key == ord('n'):
                cv.imwrite("%s/cracks/%s" % (image_folder, images), img)
                print("crack")
            else:
                filename="%s/non_cracks/%s" % (image_folder,images)
                cv.imwrite(filename, img)


for folders in range(0,6):
    sort("Image_aquisistion/Images_%s"% folders)

        

    

