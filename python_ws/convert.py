#!/usr/bin/env python3


from ultralytics import YOLO

model = YOLO("runs/segment/train2/weights/last.pt")
model.to("cuda")

model.export(format="engine", device=0)