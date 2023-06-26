from ultralytics import YOLO
import matplotlib
import matplotlib.pyplot as plt
from PIL import Image
from shapely.geometry import Polygon
import numpy as np

model = YOLO(r"C:\Users\lachl\Documents\python\test_ws\runs\segment\train5\weights\last.pt")

result = model(r"C:\Users\lachl\Documents\python\test_ws\WIN_20230510_16_56_08_Pro.jpg", conf=0.05)

im = np.asarray(Image.open(r"C:\Users\lachl\Documents\python\test_ws\WIN_20230510_16_56_13_Pro.jpg"))

poly = Polygon(result[0].masks.segments[0])
x,y = np.array(poly.exterior.xy)

# set the matplotlib backend because YOLO changes it to GUI-less
matplotlib.use("TkAgg")
plt.imshow(im)
plt.plot(x*im.shape[1], y*im.shape[0])
plt.show()