#!/usr/bin/env python3


from ultralytics import YOLO

model = YOLO("yolov8n-seg.pt")
model.to("cuda")
results = model.train(
        batch=8,
        device=0, # cuda
        data="data.yaml",
        epochs=100,
    )

model.export("engine")