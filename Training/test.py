def resize(image,percentage):
    import cv2 as cv
    scale_percent = percentage # percent of original size
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
  
    # resize image
    resized = cv.resize(image, dim, interpolation = cv.INTER_AREA)
    return resized
 


def collage(Images):
    from os import listdir
    from os.path import join
    import cv2 as cv
    import numpy as np
    list=listdir(Images)
    for images in range(0, int(len(list)/3)):
        path2annote=join(Images,  "%s.png" % images)
        path2pred=join(Images, "pred_%s.png" % images)
        path2original=join(Images, "original_%s.png" % images) 
        path2annote=path2annote.replace("\\","/")
        path2pred=path2pred.replace("\\", "/")
        path2original=path2original.replace("\\","/")

        annote=cv.imread(path2annote)
        pred=cv.imread(path2pred)
        original=cv.imread(path2original)
        
        kernel = np.ones((5,5),np.uint8)
        pred = cv.morphologyEx(pred, cv.MORPH_OPEN, kernel)
        crack = cv.bitwise_and(original, pred)
        cv.waitKey(0)

        cv.imshow("annote",resize(annote, 30))
        cv.imshow("Prediction",resize(crack, 30))
        cv.imshow("Original",resize(original, 30))

path = "Training/Images"
collage(path)
