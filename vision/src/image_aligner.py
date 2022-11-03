import numpy as np
import cv2

##Function to align images using the CPU
# Debuggin code has been removed
# Link to original: https://github.com/abmoRobotics/P5/blob/796fd9765d8d9c8c51a52b2478dc2f781e506d38/Vision/imageAligner.py#L90
def image_aligner_cpu(img1, img2):
    h,w = img1.shape

    MAX_FEATURES = 5000
    sift = cv2.ORB_create(MAX_FEATURES)

    # find the key points and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)

    match = cv2.BFMatcher()

    good = []
    try:
        matches = match.knnMatch(des1,des2,k=2)

        for m,n in matches:
            if m.distance < 0.5 * n.distance:
                good.append(m)
    except:
        print("ERROR DESCRIPTORS NOT AVAILABLE")
        return h,0
    
    #Debug to show alignment
    #draw_params = dict(matchColor = (0,255,0), # draw matches in green color
    #                singlePointColor = None,
    #                flags = 2)
    #img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
    #cv2.imshow("original_image_drawMatches.jpg", img3)
            
    MIN_MATCH_COUNT = 10
    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts, M)
        traveled = abs(np.int32(dst[0][0][1]))
        overlapHeight = h - traveled
        #print("Height image: ", h, " Traveled ", traveled, "Overlap height: ", overlapHeight)
        return traveled, overlapHeight
    else:
        print("Not enought matches are found - %d/%d", (len(good)/MIN_MATCH_COUNT))
        return h,0
