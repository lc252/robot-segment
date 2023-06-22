#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageFilter
from pathlib import Path
from scipy.ndimage import binary_closing
from rasterio import features
from shapely.geometry import Polygon
import random



def get_mask(rob):
    rob = np.array(rob)
    rob_mask = np.zeros(rob.shape[0:2])
    for row in range(480):
        for col in range(640):
            if np.all(rob[row][col]) != np.all([0,255,0]):
                rob_mask[row][col] = 1
    return rob_mask

def impose_background(rob, rob_mask, background_path):
    # load
    bkgd = np.array(Image.open(background_path).resize((640,480)))
    rob = np.array(rob)

    # put rob on background according to mask
    for row in range(480):
        for col in range(640):
            if rob_mask[row][col]:
                # rob is rgba
                bkgd[row][col] = rob[row][col][0:3]
    
    # # Debug Display
    # plt.imshow(Image.fromarray(bkgd))
    # plt.show()

    return Image.fromarray(bkgd)

def create_label(rob_mask, label_path : Path):
    rob_mask = binary_closing(rob_mask, iterations=2).astype(np.int16)

    # fit bounding polygon to mask, return list of coordinates
    cords = list(features.shapes(rob_mask, mask=(rob_mask > 0), connectivity=8))[0][0]['coordinates'][0]

    # # Display for debug
    # poly = Polygon(cords)
    # x,y = poly.exterior.xy
    # plt.imshow(rob)
    # plt.plot(x,y)
    # plt.show()

    # create yolo-compatible annotation of bounding polygon coordinates in format: "[class id] [x1 y1 x2 y2 ...]" where x and y are normalised (0-1)
    # The first "0" is the class label, here it is hardcoded as there is only one class
    label_line = '0 ' + ' '.join([f'{int(cord[0])/rob_mask.shape[1]} {int(cord[1])/rob_mask.shape[0]}' for cord in cords])

    # write annotation file
    label_path.parent.mkdir( parents=True, exist_ok=True )
    with label_path.open('w') as f:
        f.write(label_line)



if __name__ == "__main__":

    PATH = "/home/lachl/images"
    
    background_dir = list(Path(f"{PATH}/background_images").iterdir())

    # iterate each image in train, val, test sets
    for sets in ['train', 'val', 'test']:
        images_dir = Path(f"{PATH}/datasets_mask/{sets}/images")
        for img_path in images_dir.iterdir():
            # load robot with green background
            rob = Image.open(img_path)
            # rotate robot randomly
            rob = rob.rotate(random.randint(-30,30))
            # get mask
            rob_mask = get_mask(rob)
            # put robot on random background
            rob = impose_background(rob, rob_mask, random.choice(background_dir))
            # apply gaussian blur to im_arr
            rob = rob.filter(ImageFilter.GaussianBlur(radius=1))
            # save image
            # plt.imshow(rob)
            # plt.show()
            plt.imsave(f"{PATH}/datasets/{sets}/images/{img_path.stem}.png", np.array(rob))
            # save annotation
            label_path = Path(f"{PATH}/datasets/{sets}/labels/{img_path.stem}.txt")
            create_label(rob_mask, label_path)

            print("Completed:", img_path, label_path)