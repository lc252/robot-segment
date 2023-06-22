#!/usr/bin/env python3


from ultralytics import YOLO

model = YOLO("runs/segment/segsmall/weights/best.pt")
model.to("cuda")
results = model.train(
        batch=8,
        device=0, # cuda
        data="data.yaml",
        epochs=100,
        # resume=True
    )

# model.export(format="engine")