def make_crackfolder(maskfolder,rgbfolder,outputfolder):
    import os
    import cv2 as cv
    list=os.listdir(maskfolder)
    rgbPath="%s/RGB" % outputfolder
    maskPath="%s/MASK" % outputfolder
    os.mkdir(rgbPath)
    os.mkdir(maskPath)
    for images in list:
        if images.endswith(".png"):
            path1="%s/%s" % (maskfolder,images)
            path2="%s/%s" % (rgbfolder,images)
            print(path1)
            maskimg=cv.imread(path1)
            rgbimg=cv.imread(path2)

            maskpath1=os.path.join(maskPath,images)
            rgbPath1=os.path.join(rgbPath,images)
            maskpath1=maskpath1.replace("\\","/")
            rgbPath1=rgbPath1.replace("\\","/")
            print(rgbPath1)
            print(maskpath1)
            cv.imwrite(rgbPath1, rgbimg)
            cv.imwrite(maskpath1, maskimg)

def count_cracks(Image_Folder,Maskfolder):
    import os
    import cv2 as cv 
    counter=0
    list=os.listdir(Image_Folder)
    for images in list:
        path=os.path.join(Image_Folder,images)
        maskpath=os.path.join(Maskfolder,images)
        path=path.replace("\\","/")
        maskpath=maskpath.replace("\\","/")
        img=cv.imread(maskpath)
        mean=cv.mean(img)
        mean=(mean[0]+mean[1]+mean[2])/3
        print(mean)
        if mean==0:
            os.remove(maskpath)
            os.remove(path)
def show_cracks(Image_Folder,Maskfolder):
    from os.path import join

    import cv2 as cv 
    counter=0
    list=os.listdir(Image_Folder)
    for images in list:
        path = join(Image_Folder,images)
        maskpath = join(Maskfolder,images)
        path = path.replace("\\","/")
        maskpath = maskpath.replace("\\","/")
        img = cv.imread(path)
        mask_img = cv.imread(maskpath)
        cv.waitKey(0)
        cv.imshow("original",img)
        cv.imshow("Mask",mask_img)
def length_of_folder(folder):
    from os import listdir
    return len(listdir(folder))


ImageFolder= "Image_aquisistion/Images_lang2"
Maskfolder= "Image_aquisistion/Mask"
OutPutFolder= "Image_aquisistion/dadis_annotes/RGB"
OutPutFolder1= "Image_aquisistion/Wellats_annotes/RGB"
OutPutFolder2="Image_aquisistion/Marks_annotes/RGB"

#make_crackfolder(Maskfolder,ImageFolder,OutPutFolder)
#count_cracks(OutPutFolder,OutPutFolder1)
#show_cracks(OutPutFolder,OutPutFolder1)
size_of_annotates=length_of_folder(OutPutFolder)+length_of_folder(OutPutFolder1)+length_of_folder(OutPutFolder2)
test_set=int(size_of_annotates*0.2)
training_set=int(size_of_annotates*0.8)
print(training_set)
print(test_set)

