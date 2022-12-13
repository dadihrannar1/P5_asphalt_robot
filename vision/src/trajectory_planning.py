import cv2
import time
import numpy as np
from skimage.morphology import skeletonize
from skimage import measure
import networkx as nx
from sklearn.neighbors import NearestNeighbors
from scipy.spatial import distance
from scipy import signal
import warnings


#TODO this class should be cleaned up
class Crack:
    # Coordinates of all cracks in raw pixels
    _crack_coordinates_raw = []
    # Current index of cracks_coordinates
    _index = 0
    # Checks whether a crack is finished
    _is_done = False
    # number of coordinates
    len = 0
    
    def __init__(self,crack_cords) -> None:
        self._crack_coordinates = crack_cords
        self._add_state()
        self.len = len(self._crack_coordinates)
        self._crack_in_next_frame = False
    
    def __len__(self):
        return len(self._crack_coordinates)

    def __getitem__(self,index):
        return self._crack_coordinates[index]

    # Mark ends of cracks
    def _add_state(self):
        for idx in range(0,len(self._crack_coordinates)):
            self._crack_coordinates[idx].append(False)
        self._crack_coordinates[0][2] = True
        self._crack_coordinates[-1][2] = True

    #UNUSED
    def set_crack_in_next_frame(self):
        self._crack_in_next_frame = True

    #UNUSED
    def crack_in_next_frame(self):
        return self._crack_in_next_frame

    def get_first_crack(self):
        return self._crack_coordinates[0]
    
    def shift(self):
        self._crack_coordinates[self._index][2] = True

    def get_coordinates(self):
        return self._crack_coordinates

    def get_last_crack(self):
        return self._crack_coordinates[-1]
    
    def get_current_crack(self):
        return self._crack_coordinates[self._index]


    def repair(self):
        if not (self._crack_coordinates[self._index] == self._crack_coordinates[-1]):
            self._index += 1
        else:
            self._is_done = True
        

    def get_index(self):
        return self._index

    def is_done(self):
        return self._is_done

    # Removes some coordinates to ensure that 
    def _sort_crack():
        pass


#TODO this class should be cleaned up
class Frame():
    # Pixel on the y-axis where the next frame begins
    next_frame = 480
    
    KEEP_EVERY_X_PIXEL = 3
    # all cracks done
    cracks_done = False

    def __init__(self) -> None:
        self.path = []
        self.raw_cracks: Crack = []
        self.cracks: Crack  = []
        self.frame_time = 0
        pass
    def __len__(self):
        return len(self.cracks)
    # Add crack to path planning
    def add_crack(self, crack: Crack) -> Crack:
        # Check whether input data is right format
        if not isinstance(crack, Crack):
            raise TypeError("Input type must be crack")
        else:       
            # check if crack is in next frame
            #last_crack = crack.get_last_crack()
            #if ((crack.get_last_crack())[1] > self.next_frame):
            #    crack.set_crack_in_next_frame()

            # Add crack to frame
            self.raw_cracks.append(crack)
    # 
    def cracks_done(self):
        length = len(self.raw_cracks)
        counter = 0
        
        for cracker in self.raw_cracks:
            if cracker.is_done():
                counter = counter + 1

        if (counter == length):
            return True
        else:
            return False

    # Get crack with largest y-pixel value.
    def _largest_y_value(self):
        # Find start point
        max_pixel = 99999
        # Starting object
        obj_index = 0
        for idx, crack in enumerate(self.cracks):

            # Get the last crack coordinates that has been repaired
            if not crack.is_done():
                last_repaired_coordinate = crack.get_current_crack()
                # Check whether y pixel is less than previous
                if max_pixel > last_repaired_coordinate[1]:
                    max_pixel = last_repaired_coordinate[1]
                    obj_index = idx

        return max_pixel, obj_index

    # Checks whether the next frame is ready
    def _next_frame_ready(self):
        max_pixel, obj_index = self._largest_y_value()
        if (max_pixel > self.next_frame):
            return True
        else: 
            return False

    def _remove_pixels(self):
        resolution = self.KEEP_EVERY_X_PIXEL
        
        self.cracks.clear()

        for crack in self.raw_cracks:
            temp_crack = []
            for idx in range(0, len(crack), resolution):
                temp_crack.append(crack[idx])
            
            c = Crack(temp_crack)
            self.cracks.append(c)
            


    def get_json_array(self):
        json_array = []

        for path in self.path:
            Data = {
                "Position": {
                    "X": path[0],
                    "Y": path[1]
                    },
                "Time": {"Detected": self.frame_time},
                "Crack": {"DetectionIndex": path[2]}
                }
            
            json_array.append(Data)
        
        return json_array

    def find_path(self):
        # Remove path if already calculated
        self.path.clear()
        # Save evey third pixel
        self._remove_pixels()
        PIXEL_INCREMENT = 96*1.55
        while not self._next_frame_ready() and not self.cracks_done():
            
            # Get the object where the first crack begins
            max_pixel, obj_index = self._largest_y_value()
            
            #while not frame.cracks[obj_index].is_done():
            for idx in range(
                self.cracks[obj_index].get_index(), # Get the current crack not repaired
                self.cracks[obj_index].len          # Run for loop until last object is reached
                ):
                max_pixel2, obj_index2 = self._largest_y_value()
                # If next frame is ready exit for loop and continue with next frame
                if self._next_frame_ready():
                    break
                # Check if should shift to another crack
                elif (((self.cracks[obj_index].get_current_crack()[1]) > (max_pixel2 + PIXEL_INCREMENT)) or (self.cracks[obj_index]._is_done)):
                    self.cracks[obj_index].shift()
                    self.path.append(self.cracks[obj_index].get_current_crack())
                    #print("From", self.cracks[obj_index].get_current_crack())
                    self.cracks[obj_index2].shift()
                    #print("To", self.cracks[obj_index2].get_current_crack())
                    obj_index = obj_index2
                    break
                # Point will be repaired
                else:
                    self.path.append(self.cracks[obj_index].get_current_crack())
                    self.cracks[obj_index].repair()
            

    def set_frame_time(self, t):
        self.frame_time = t

    def get_frame_time(self):
        return self.frame_time


# Determines which cracks must be transfered
def map_cracks(frame1: Frame, frame2: Frame, offset: int):
    '''Maps cracks from frame 1 to frame 2'''

    # First the function maps crack from frame 1 to frame 2.
    mapped_cracks = []
    for i, crack_frame1 in enumerate(frame1.raw_cracks):
        if crack_frame1.crack_in_next_frame():
            x_val_frame1 = (crack_frame1.get_last_crack())[0]
            index = 0
            min_val = 999
            for idx, crack_frame2 in enumerate(frame2.raw_cracks):
                x_val_frame2 = (crack_frame2.get_first_crack())[0]
                frame_diff = abs(x_val_frame2-x_val_frame1) 
                if frame_diff < min_val:
                    min_val = frame_diff
                    index = idx
            mapped_cracks.append({"old_frame": i, "new_frame": index})

    # Secondly the function marks the transfered "repaired" cracks from frame 1 to frame 2 and marks them as repaired
    for cracks in mapped_cracks:
        old_crack_id = cracks['old_frame']
        new_crack_id = cracks['new_frame']

        frame1_current_crack = frame1.raw_cracks[old_crack_id].get_current_crack()
        while not frame2.raw_cracks[new_crack_id].is_done():
            if ((frame2.raw_cracks[new_crack_id].get_current_crack())[1] <= frame1_current_crack[1] - offset):
                # Marks the current point in the crack as "repaired"
                frame2.raw_cracks[new_crack_id].repair()
            else:
                # Indicates that this is the start/end of a crack
                frame2.raw_cracks[new_crack_id].shift()
                break

    return mapped_cracks


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


def sort_branch(regions):
    crack_cords = []

    for region in regions:
        points = region
       
        clf = NearestNeighbors(radius=1.5,metric='euclidean').fit(points)
       
        P = clf.radius_neighbors_graph() 
        
        T = nx.from_scipy_sparse_matrix(P)
        end_points =[]
        index = []
        for idx, t in enumerate(T):
            if len(T[idx]) < 2:
                end_points.append((points[idx][1]))
                index.append(idx)
              
        maxer = max(end_points)
        
        start_index = end_points.index(maxer)
        
        order = list(nx.dfs_preorder_nodes(T, index[start_index-1]))
        
        sorted_crack = [list(region[i]) for i in order]
        #print('beginning\n')
        #print(sorted_crack)  
        crack_cords.append(sorted_crack)

    return crack_cords


def process_image(img):
    # Get skeleton
    skeleton = skeletonize(img) # 12 ms
    
    # Extract array with pixels from skeleton
    skeleton_points = np.where(skeleton[:]==1) # 1 ms
    
    # Convert to (x, y)
    skeleton_points = list(zip(skeleton_points[1], skeleton_points[0])) # 0 ms

    # Find branches
    skeleton_branches = find_branches(skeleton, skeleton_points) # 5 ms
    
    # Find regions
    regions,reg2 = region_array(skeleton_branches,True) # 3 ms

    # Sort branches
    sorted_cracks = sort_branch(regions) # 9 ms

    return sorted_cracks


# Function to calculate the length the robot needs to travel to follow a trajectory
def calculate_trajectory_length(trajectory: np.array, current_pos: np.array):
    # Turn trajectory from strings to proper format
    trajectory = np.column_stack([trajectory[:, 0:2].astype(int), trajectory[:, 2]])

    # Calculate distance between all points in trajectory
    dist_between_points = distance.cdist(trajectory[:, 0:2], trajectory[:, 0:2], metric='euclidean').diagonal(1)

    # Calculate distance from current pos and ad this distance to total
    beginning_dist = ((current_pos[0] - trajectory[0, 0]) ** 2 + (current_pos[1] - trajectory[0, 1]) ** 2) ** 0.5
    dist_with_beginning = np.append(np.array([beginning_dist]), dist_between_points)

    # Remove redundant true values to allow calculating the distance of cracks and between cracks separately
    # 1  0  0  1  1  0  0  1
    # 1  0  0  0  1  0  0  0

    # Turns instances of ...0 1 1 0... into ...0 1 0 -1... and ...0 1 0... into ...0 1 -1...
    kernel = np.array([-1, 1])
    convolved = signal.convolve(trajectory[:, 2], kernel, mode='full')

    # Setting all -1 values to 0
    crack_bool = np.where(convolved[1:] >= 0, convolved[1:], 0)
    crack_bool[-1] = 0  # The end of the trajectory should always be considered part of a crack

    # Sum the distances
    crack_length = np.sum(np.where(crack_bool[:] == 0, dist_with_beginning, 0)).astype(int)
    between_length = np.sum(np.where(crack_bool[:] == 1, dist_with_beginning, 0)).astype(int)

    return crack_length, between_length
