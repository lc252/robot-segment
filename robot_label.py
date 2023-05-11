from rasterio import features
import numpy as np
from PIL import Image
from pathlib import Path
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from scipy.ndimage import binary_closing


def create_label(image_path, label_path):
    arr = np.asarray(Image.open(image_path))
    rob_mask = np.zeros(arr.shape[0:2])

    for row in range(len(arr)):
        for col in range(len(arr[row])):
            if np.all(arr[row][col]) != np.all([0,255,0]):
                rob_mask[row][col] = 1

    rob_mask = binary_closing(rob_mask, iterations=2).astype(int)

    cords = list(features.shapes(rob_mask, mask=(rob_mask > 0), connectivity=8))[0][0]['coordinates'][0]

    # poly = Polygon(cords)
    # x,y = poly.exterior.xy
    # plt.imshow(rob_mask)
    # plt.plot(x,y)
    # plt.show()

    label_line = '0 ' + ' '.join([f'{int(cord[0])/arr.shape[1]} {int(cord[1])/arr.shape[0]}' for cord in cords])

    label_path.parent.mkdir( parents=True, exist_ok=True )
    with label_path.open('w') as f:
        f.write(label_line)

for images_dir_path in [Path(f'datasets/{x}/images') for x in ['train', 'val', 'test']]:
    for img_path in images_dir_path.iterdir():
        label_path = img_path.parent.parent / 'labels' / f'{img_path.stem}.txt'
        print(img_path)
        label_line = create_label(img_path, label_path)