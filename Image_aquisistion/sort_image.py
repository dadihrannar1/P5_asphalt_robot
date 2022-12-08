import json

def remove_image(damage_list, image_path):
    import os
    images=iter(image_path)
    for folders in range(0,len(image_path)):
        path=next(images)
        print(folders)
        for elements in damage_list[folders]:
            img_path=os.path.join(path, elements)
            img_path=img_path.replace("\\","/")
            if img_path.endswith(".png"):
                os.remove(img_path)

def mark_braking_images(path):
    import cv2 as cv
    import os
    image_names = os.listdir(path)
    destroyed_images=[]
    for images in image_names:
        if images.endswith(".png"):
            pth_2_image="%s/%s" % (path, images)
            img = cv.imread(pth_2_image)
            ROI=img[721:812, 1337:1493]
            GoproROI=img[1018:1077, 838:874]
            ave=cv.mean(ROI)
            ave=(ave[0]+ave[1]+ave[2])/3
            GoAve=cv.mean(GoproROI)
            GoG=GoAve[1]
            GoB=GoAve[0]
            if ave < 170:
                destroyed_images.append(images)
            if 150 < GoG < 162 and 210 < GoB < 225:
                destroyed_images.append(images)
    return destroyed_images
        
Image_folder = ["Image_aquisistion/Images_lang2","Image_aquisistion/Images_lang3","Image_aquisistion/Images_f_1","Image_aquisistion/Images_f_2","Image_aquisistion/Images_f_3","Image_aquisistion/Images_f2_1","Image_aquisistion/Images_f2_2","Image_aquisistion/Images_f2_3"]

images=iter(Image_folder)

full_list = []
for folders in range(0,6):
    ruined_images = mark_braking_images(next(images))
    full_list.append(ruined_images)
print(full_list)

remove_image(full_list, Image_folder)

'''
json_output= []
for folders in range(0,6):
    f=open("Image_aquisistion/image_details_%s.json" % folders,"r")
    read_data = json.load(f)
    for elements in full_list[folders]:
        if folders == 0:
            path = "Images/%s" % elements
        elif folders == 2:
            path = "Images2qqqq/%s" % elements  
        else:
            path="Images%s/%s" % (folders, elements)
        index=read_data[0].index(path)
        read_data[0].pop(index)
        read_data[1].pop(index)
        read_data[2].pop(index)
        read_data[3].pop(index)
    if len(read_data[0])==len(read_data[1])==len(read_data[2])==len(read_data[3]):
        json_output.append(read_data)
    dump = json.dumps(json_output)
    open("Image_aquisistion/edited_image_details_%s.json" % folders, "x")
    with open("Image_aquisistion/edited_image_details_%s.json" % folders, "w") as outfile:
        outfile.write(dump)

        
remove_image(full_list)
'''










