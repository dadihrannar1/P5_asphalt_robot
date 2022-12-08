def cut_images(RGB,Mask,output_folder):
    import os
    import cv2 as cv
    import numpy as np
    list=os.listdir(RGB)
    for images in list:
        path2img=os.path.join(RGB,images)
        path2mask=os.path.join(Mask,images)
        img=cv.imread(path2img)
        mask_img=cv.imread(path2mask)

        nonskewed=img[963:988, 328:351]
        skewed=img[749:768, 1399:1449]
        skewedave=cv.mean(skewed)
        skewedave=(skewedave[0]+skewedave[1]+skewedave[2])/3
        if skewedave > 230:
            pts = np.array([[484,310],[1244,195],[1350,769],[571,893]])
            ## (1) Crop the bounding rect
            rect = cv.boundingRect(pts)
            x,y,w,h = rect
            croped = img[y:y+h, x:x+w].copy()
            cropedmask=mask_img[y:y+h, x:x+w].copy()

            ## (2) make mask
            pts = pts - pts.min(axis=0)

            mask = np.zeros(croped.shape[:2], np.uint8)
            cv.drawContours(mask, [pts], -1, (255, 255, 255), -1, cv.LINE_AA)

            ## (3) do bit-op
            dst = cv.bitwise_and(croped, croped, mask=mask)
            dst1=cv.bitwise_and(cropedmask, cropedmask, mask=mask)
            cv.imwrite("%s/RGB/%s" % (output_folder,images), dst)
            cv.imwrite("%s/Masks/%s" % (output_folder,images), dst1)
        else:
            pts = np.array([[437,154],[1595,154],[1587,983],[421,968]])
            ## (1) Crop the bounding rect
            rect = cv.boundingRect(pts)
            x,y,w,h = rect
            croped = img[y:y+h, x:x+w].copy()
            cropedmask=mask_img[y:y+h, x:x+w].copy()

            ## (2) make mask
            pts = pts - pts.min(axis=0)

            mask = np.zeros(croped.shape[:2], np.uint8)
            cv.drawContours(mask, [pts], -1, (255, 255, 255), -1, cv.LINE_AA)

            ## (3) do bit-op
            dst = cv.bitwise_and(croped, croped, mask=mask)
            dst1=cv.bitwise_and(cropedmask, cropedmask, mask=mask)
            cv.imwrite("%s/RGB/%s" % (output_folder,images), dst)
            cv.imwrite("%s/masks/%s" % (output_folder,images), dst1)
            


cut_images("Training/Data/Train_data/RGB","Training/Data/Train_data/Masks","Training/New_data")