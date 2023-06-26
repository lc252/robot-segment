import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from pathlib import Path
from scipy.ndimage import binary_closing
import random



def impose_rob(rob_path, background_path):
    # load
    im = Image.open(background_path)
    im_arr = np.array(im.resize((640,480)))

    rob = np.array(Image.open(rob_path))
    rob_mask = np.zeros(rob.shape[0:2])

    # make mask from green
    for row in range(480):
        for col in range(640):
            if np.all(rob[row][col]) != np.all([0,255,0]):
                rob_mask[row][col] = 1

    # only use this line in labelling because polys must be enclosed
    # rob_mask = binary_closing(rob_mask, iterations=2).astype(int)

    # put rob on back according to mask
    for row in range(480):
        for col in range(640):
            if rob_mask[row][col]:
                # rob is rgba
                im_arr[row][col] = rob[row][col][0:3]
    #render
    print("datasets"+str(rob_path)[12:])
    plt.imsave("datasets"+str(rob_path)[12:], im_arr)
    # plt.imshow(im_arr)
    # plt.show()

if __name__ == "__main__":
    background_dir = list(Path("background_images").iterdir())
    for images_dir_path in [Path(f'mask_dataset/{x}/images') for x in ['train', 'val', 'test']]:
        for img_path in images_dir_path.iterdir():
            impose_rob(img_path, random.choice(background_dir))