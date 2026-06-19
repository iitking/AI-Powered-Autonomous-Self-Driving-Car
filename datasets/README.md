# 📁 Autonomous Driving Datasets & Training Guide

This directory contains resources, directory layouts, and scripts for collecting custom training datasets from the vehicle's onboard camera.

---

## 1. Directory Structure

```text
datasets/
├── raw_frames/             # Unlabeled video frames captured from vehicle camera
│   ├── day_run_01/
│   └── night_run_01/
│
├── lane_dataset/           # Binary masks & polynomial coordinates
│   ├── images/
│   └── masks/
│
├── object_detection/       # YOLO format annotations
│   ├── images/
│   │   ├── train/
│   │   └── val/
│   └── labels/
│       ├── train/
│       └── val/
│
└── data.yaml               # YOLO dataset configuration file
```

---

## 2. YOLO `data.yaml` Specification

```yaml
path: ../datasets/object_detection
train: images/train
val: images/val

names:
  0: person
  1: car
  2: bicycle
  3: traffic_light
  4: stop_sign
  5: cone
  6: obstacle_box
```

---

## 3. Training Custom YOLOv8 Model

To fine-tune a YOLOv8 nano model on your custom vehicle dataset:

```bash
yolo task=detect mode=train model=yolov8n.pt data=datasets/data.yaml epochs=50 imgsz=640 batch=16
```

Once training completes, export the weights to ONNX or TensorRT for optimized edge inference on NVIDIA Jetson:

```bash
yolo export model=runs/detect/train/weights/best.pt format=engine device=0
```
