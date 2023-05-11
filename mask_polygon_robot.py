from rasterio import features
import numpy as np
from PIL import Image
from pathlib import Path
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from scipy.ndimage import binary_closing


arr = np.asarray(Image.open("robot.png"))
rob_mask = np.zeros(arr.shape[0:2])

for row in range(len(arr)):
    for col in range(len(arr[row])):
        if np.all(arr[row][col]) != np.all([0,255,0]):
            rob_mask[row][col] = 1

rob_mask = binary_closing(rob_mask, iterations=2).astype(int)

# There may be a better way to do it, but this is what I have found so far
cords = list(features.shapes(rob_mask, mask=(rob_mask > 0), connectivity=8))[0][0]['coordinates'][0]
print(cords)
poly = Polygon(cords)
x,y = poly.exterior.xy
plt.imshow(rob_mask)
plt.imsave("robot_mask.png", rob_mask)
plt.plot(x,y)
plt.show()
