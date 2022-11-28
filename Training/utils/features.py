
from os import environ
import cv2
from PIL import Image
from networkx.algorithms import hierarchy
import numpy as np
from skimage.morphology import medial_axis, skeletonize, binary_closing
from skimage import data, measure
import matplotlib.pyplot as plt
from skimage.util import invert
import time
import networkx as nx
from sklearn.neighbors import NearestNeighbors, radius_neighbors_graph
import math

#from ..path_planning import frame

def closing(img):
    #st1 = time.time()
    image = binary_closing(img, selem=np.ones((9, 9)))
    #stop = time.time()
    #print("Time-Morph: ", stop-st1)
    # kernel = np.ones((19,19),np.uint8)
    # image = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    #print(image.shape)
    # start = time.time()
    image = image.astype(np.uint8)
    
    cnts, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    #start = time.time()
    mask = np.zeros(image.shape[:2], dtype="uint8")
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        #perimeter = cv2.arcLength(cnt,True)
        #circularity = 4*math.pi*(area/(perimeter*perimeter))
        # print("Perimeter: ", perimeter)
        # print("Circularity: ", circularity)
        # print("Area: ", area)
        if area > 200:
             cv2.drawContours(mask, [cnt], 0, (255), -1)

    result = cv2.bitwise_and(image,image, mask= mask)
    # end = time.time()
    # print("Time:", end-start)
    # print("FPS: ", str(1/(time.time()-start)))
    return result


def skeletonization(img):
    skeleton = skeletonize(img)
    return skeleton

def find_length_of_skeletonization(img):
    pass

# Function for calculating the avergage width of a crack
def crack_width(img):
    contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # create an empty black image
    drawing = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)

    # draw contours and hull points
    for i in range(len(contours)):
        print(cv2.contourArea(contours[i]))

        color_contours = (255,255,255) # green - color for contours
        # draw ith contour
        cv2.drawContours(drawing, contours, i, color_contours, 1, 8, hierarchy)

    return contours

# # Billede vi loader for at teste
def test_skeletonize():
    img = Image.open(r"C:\P5\P5\Vision\data\train_masks\20160328_151013_361_1281.png").convert('L')
    img = np.array(img)
    (thresh, blackAndWhiteImage) = cv2.threshold(img, 127, 1, cv2.THRESH_BINARY)


    contours = closing(blackAndWhiteImage)


    s1 = skeletonize(contours)


    fig, axes = plt.subplots(1,2, figsize=(8, 8), sharex=True, sharey=True)
    ax = axes.ravel()



    ax[0].imshow(img, cmap=plt.cm.gray)
    ax[0].set_title('original')
    ax[0].axis('off')

    ax[1].imshow(s1, cmap=plt.cm.gray)
    ax[1].set_title('skeleton')
    ax[1].axis('off')

    fig.tight_layout()
    plt.show()


    # print(s1.shape)
    # binary = (s1*255).astype(np.uint8)
    # while 1:
    #     cv2.imshow("asdf",binary)
    #     cv2.waitKey(20)

def skeletonization_with_threshold(img):
    (thresh, blackAndWhiteImage) = cv2.threshold(img, 127, 1, cv2.THRESH_BINARY)

    contours = closing(blackAndWhiteImage)
    skeleton = skeletonize(contours)



    return skeleton

def region_array(img, xyFomrat=True):
    labels, num = measure.label(img,connectivity=2,return_num=True)
    MINIMUM_AREA = 2
    region_arr =[]
    region_arr_xy =[]
    for i in range(0,num):
        region = np.where((labels[:]==i+1))
        region_arr_xy.append(region)

        if len(region[0]) > MINIMUM_AREA:
            # (x,y)
            region = list(zip(region[1], region[0]))
            region_arr.append(region)

        
    return region_arr, region_arr_xy

def find_branches(image, preds):
    kernel = np.array( [[1, 1, 1],
                        [1, 10, 1],
                        [1, 1, 1]])
    image = (image).astype(np.uint8)
	# grab the spatial dimensions of the image, along with
	# the spatial dimensions of the kernel
    (iH, iW) = image.shape[:2]
    (kH, kW) = kernel.shape[:2]
	# allocate memory for the output image, taking care to
	# "pad" the borders of the input image so the spatial
	# size (i.e., width and height) are not reduced
    pad = (kW - 1) // 2
    output = image.copy()
    image = cv2.copyMakeBorder(image, pad, pad, pad, pad,
        cv2.BORDER_REPLICATE)


    for point in preds:
        x = point[0]+pad
        y = point[1]+pad
        roi = image[y - pad:y + pad + 1, x - pad:x + pad + 1]
        k = (roi * kernel).sum()
        if k > 12:
                output[y - pad, x - pad] = 0
        else:
            output[y - pad, x - pad] = 1

    return output

def sort_branch(regions):
    crack_cords = []

    # find ends

    for region in regions:
        points = region
       # clf2 = radius_neighbors_graph()
        #clf = NearestNeighbors(n_neighbors=2,algorithm='auto').fit(points)
        clf = NearestNeighbors(radius=1.5,metric='euclidean').fit(points)
       # print(clf.radius_neighbors)
        print("HEJ")
        #G = clf.kneighbors_graph()
        P = clf.radius_neighbors_graph() 
        #print("HEK", G)
        
        T = nx.from_scipy_sparse_matrix(P)
        end_points =[]
        index = []
        for idx, t in enumerate(T):
            #print(T[idx])
            #print(idx)
            if len(T[idx]) < 2:
                end_points.append((points[idx][1]))
                index.append(idx)
              #print(points[idx])
        #print(end_points)
        maxer = max(end_points)
        #print("hejhej", end_points.index(maxer))
        start_index = end_points.index(maxer)
        print(index[start_index])
        print(index[0])
        print(index[1])
        #print(len(T[0]))
        order = list(nx.dfs_preorder_nodes(T, index[start_index-1]))
        #print(region)
        sorted_crack = [list(region[i]) for i in order]
        #region = region[order]
        crack_cords.append(sorted_crack)
        
    return crack_cords

# Input must be binary
def process_image(img):

    # Remove holes in image
    t1 = time.time()
    #img = closing(img) # 5 ms
    cv2.imshow("Morph", img.astype(np.uint8)*255)
    cv2.waitKey(10)
    # Get skeleton
    
    skeleton = skeletonize(img) # 12 ms
    cv2.imwrite("hej.png", (skeleton*255).astype(np.uint8))
    # Extract array with pixels from skeleton
    skeleton_points = np.where(skeleton[:]==1) # 1 ms
    
    # Convert to (x, y)
    skeleton_points = list(zip(skeleton_points[1], skeleton_points[0])) # 0 ms

    # Find branches
    skeleton_branches = find_branches(skeleton, skeleton_points) # 5 ms
    
    # Find regions
    regions,reg2 = region_array(skeleton_branches,True) # 3 ms
    if(0):
        y1 = reg2[3][0]
        x1 = reg2[3][1]
        
        plt.plot(x1, 940-y1)
        y2 = reg2[2][0]
        x2 = reg2[2][1]

        plt.plot(x2, 940-y2)
        y3 = reg2[4][0]
        x3 = reg2[4][1]

        plt.plot(x3, 940-y3)
        plt.xlim([0, 540])
        plt.ylim([0, 940])

        plt.show()
    # Sort regions WAITING FOR JESPER
    sorted_cracks = sort_branch(regions) # 9 ms
    # print(sorted_cracks[2])
    if(0):
        y1 = reg2[3][0][sorted_cracks[2]]
        x1 = reg2[3][1][sorted_cracks[2]]
        
        plt.plot(x1, 940-y1)
        y2 = reg2[2][0][sorted_cracks[1]]
        x2 = reg2[2][1][sorted_cracks[1]]

        plt.plot(x2, 940-y2)
        y3 = reg2[4][0][sorted_cracks[3]]
        x3 = reg2[4][1][sorted_cracks[3]]

        plt.plot(x3, 940-y3)
        plt.xlim([0, 540])
        plt.ylim([0, 940])

        plt.show()
    #print("TIME: ", str(time.time()-t1))
    print("ERHADSFHSAJLOKDHJKOSA")
    print(sorted_cracks[1])
    e = []
    e.append(sorted_cracks[0])
    e.append(sorted_cracks[1])
    return sorted_cracks

if __name__ == "__main__":
    #img = Image.open(r"C:\P5\P5\Vision\data\train_masks\20160328_151013_361_1281.png").convert('L')
    img = Image.open(r"../closing.jpg").convert('L')
    img = np.array(img)
    #img = np.rot90(img)
    (thresh, blackAndWhiteImage) = cv2.threshold(img, 127, 1, cv2.THRESH_BINARY)
    #binary_image = closing(blackAndWhiteImage)
    t1 = time.time()
    sorted_cracks = process_image(blackAndWhiteImage)
    print(time.time()-t1)
    #print(sorted_cracks[0])
    # # Get skeleton
    # skeleton = skeletonization_with_threshold(img)

    # # Extract array with pixels from skeleton
    # skeleton_points = np.where(skeleton[:]==1)

    # # Convert to (x, y)
    # skeleton_points = list(zip(skeleton_points[1], skeleton_points[0]))

    # # Find branches
    # skeleton_branches = find_branches(skeleton, skeleton_points)

    # # Find regions
    # regions = region_array(skeleton_branches)

    # # Sort regions
    # sorted_cracks = sort_branch(regions)


    # fig, axs = plt.subplots(1,2)
    # fig.suptitle('Vertically stacked subplots')
    # axs[0].imshow(skeleton)
    # axs[1].imshow(skeleton_branches)
    # # plt.imshow(skeleton)
    # # plt.imshow(skeleton2)
    # plt.axis('off')
    # plt.show()
