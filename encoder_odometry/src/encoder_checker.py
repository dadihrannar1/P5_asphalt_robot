import json
import numpy as np

path = r"C:\Users\mikip\Desktop\VM images\Dadi-image\shared files\Images_lang2"

# Open the json file and extract data
with open(f"{path}/image_details.json") as json_file:
    data = json.load(json_file)
    filename = data[0]
    timer = [int(s) for s in data[1]]  # Converting string to int
    encoder1 = [int(s) for s in data[2]]
    encoder2 = [int(s) for s in data[3]]

pixelSize = 0.8853  # mm/pixel
tickSize = 11.9381  # encoder size
tickPixel = tickSize /pixelSize  # encoder ticks to pixel size

full_length = ([],[])
for enc1, enc2 in zip(encoder1[1290:1340],encoder2[1290:1340]):
    print(enc1*tickPixel)
    circum = np.pi * 380
    wheel_offset = 1.446*1000
    enc1L = enc1*tickSize
    enc2L = enc2*tickSize
    wheelangle = ((enc1 - enc2) / 100) * (circum / wheel_offset)
    full_length[0].append((enc1 + enc2) / 2 * np.sin(wheelangle)*tickPixel)
    full_length[1].append((enc1 + enc2)/ 2 * np.cos(wheelangle)*tickPixel)
    print(f'wheel angle {wheelangle}')
    print(f"Traveled along the x axis =  {(enc1 + enc2) / 2 * np.sin(wheelangle)*tickPixel}")#
    print(f"Traveled along the y axis = {(enc1 + enc2)/ 2* np.cos(wheelangle)*tickPixel}")

print(f'full length = {sum(full_length[0])}, {sum(full_length[1])}')