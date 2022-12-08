def sort(image_folder):
    import cv2 as cv
    import os 

    os.mkdir("%s/cracks" % image_folder)
    os.mkdir("%s/non_cracks" % image_folder)
    for images in os.listdir(image_folder):
        if images !="cracks" and images != "non_cracks" and images.endswith(".png"):
            filename="%s/%s" % (image_folder,images)
            img = cv.imread(filename)
            cv.imshow("hej",img)
            key = cv.waitKey(0)
            if key == ord('n'):
                cv.imwrite("%s/cracks/%s" % (image_folder, images), img)
            else:
                filename="%s/non_cracks/%s" % (image_folder,images)
                cv.imwrite(filename, img)

Image_folder = ["Image_aquisistion/Images_lang","Image_aquisistion/Images_tur","Image_aquisistion/Images_w_1","Image_aquisistion/Images_w_2","Image_aquisistion/Images_w_3","Image_aquisistion/Images_w_4"]

images_iter=iter(Image_folder)
for folders in range(0,len(Image_folder)):
    sort(next(images_iter))

        

    

