#!/usr/bin/env python3

# combination of robot_label and impose_robot_background



import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from pathlib import Path
from scipy.ndimage import binary_closing
from rasterio import features
from shapely.geometry import Polygon
import random



def get_mask(rob):
    rob_mask = np.zeros(rob.shape[0:2])
    for row in range(480):
        for col in range(640):
            if np.all(rob[row][col]) != np.all([0,255,0]):
                rob_mask[row][col] = 1
    return rob_mask

def impose_background(rob, rob_mask, background_path, rob_path):
    # load
    im = Image.open(background_path)
    im_arr = np.array(im.resize((640,480)))

    # put rob on background according to mask
    for row in range(480):
        for col in range(640):
            if rob_mask[row][col]:
                # rob is rgba
                im_arr[row][col] = rob[row][col][0:3]
    # render
    # print("/home/fif/lc252/datasets" + str(rob_path)[28:])
    plt.imsave("/home/fif/lc252/datasets" + str(rob_path)[28:], im_arr)
    return im_arr


def create_label(rob, rob_mask, label_path):
    arr = rob # np.asarray(Image.open(image_path))

    rob_mask = binary_closing(rob_mask, iterations=2).astype(np.int16)

    cords = list(features.shapes(rob_mask, mask=(rob_mask > 0), connectivity=8))[0][0]['coordinates'][0]

    # poly = Polygon(cords)
    # x,y = poly.exterior.xy
    # plt.imshow(rob)
    # plt.plot(x,y)
    # plt.show()

    label_line = '0 ' + ' '.join([f'{int(cord[0])/arr.shape[1]} {int(cord[1])/arr.shape[0]}' for cord in cords])

    label_path.parent.mkdir( parents=True, exist_ok=True )
    with label_path.open('w') as f:
        f.write(label_line)

if __name__ == "__main__":
    
    background_dir = list(Path("/home/fif/lc252/background_images").iterdir())

    for images_dir_path in [Path(f'/home/fif/lc252/datasets_raw/{x}/images') for x in ['train', 'val', 'test']]:
        for img_path in images_dir_path.iterdir():
            # load robot with green and do random rotate
            rob = np.asarray(Image.open(img_path).rotate(random.randint(-30,30)))
            # get mask
            rob_mask = get_mask(rob)
            # put robot on random background
            rob = impose_background(rob, rob_mask, random.choice(background_dir), img_path)
            # get the label path
            label_path = images_dir_path.parent.parent.parent / "datasets" / f"{str(images_dir_path)[29:-7]}" / "labels" / f"{img_path.stem}.txt"

            create_label(rob, rob_mask, label_path)

            print(img_path, label_path)