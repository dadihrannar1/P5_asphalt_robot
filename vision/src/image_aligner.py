import numpy as np
import cv2
import math
from scipy.spatial.transform import Rotation

# Finds the affine transform between two images using the CPU
def visual_odometry(img1, img2, max_features=5000, min_match_count=10, keep_best_ratio=0.25):
    # find the key points and descriptors with SIFT
    sift = cv2.ORB_create(max_features)
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # Matching object (crosscheck to reduce false positives)
    matches = bf.match(des1, des2)  # Match descriptors
    matches = sorted(matches, key=lambda x: x.distance)  # Sort matches according to their hamming distance

    # Debug to show alignment
    # draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
    #                    singlePointColor=None,
    #                    flags=2)
    # img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:50], None, **draw_params)
    # cv2.imshow("original_image_drawMatches.jpg", img3)
    # cv2.waitKey(0)

    good = matches[:int(len(matches)*keep_best_ratio)]
    if len(good) > min_match_count:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        # Generate 2x3 transformation matrix containing rotational and translational components [R_2x2|T_2x1]
        transformation, _ = cv2.estimateAffinePartial2D(dst_pts, src_pts)

        # X and Y translations
        x_dist = transformation[0, 2]
        y_dist = transformation[1, 2]

        # Remove scaling factor from rotation matrix
        angle = math.acos(transformation[0, 0]/math.sqrt(math.pow(transformation[0, 0],2) + math.pow(transformation[0, 1],2)))

        # If angle change is above 180 degrees take it as negative angle instead
        if angle > math.pi:
            angle = angle - 2*math.pi
        return angle, x_dist, y_dist
    else:
        print("Not enough matches are found - %d/%d", (len(good) / min_match_count))
        angle = 0
        x_dist = 0
        y_dist = 0
        return angle, x_dist, y_dist


# Turn affine transform into angle and 
def affine_to_angle_trans(transformation):
    angle = (math.asin(transformation[1, 0]))/math.pi*180
    x_dist = int(transformation[0, 2])
    y_dist = int(transformation[1, 2])
    return angle, x_dist, y_dist
