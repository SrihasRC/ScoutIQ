#!/usr/bin/env python3
"""Utility script to bake offline vocabulary into YOLO-World and export to ONNX."""

import argparse
import os
from ultralytics import YOLOWorld


def export_yoloworld_onnx(
    classes=None,
    model_name="yolov8s-worldv2.pt",
    output_onnx="/home/srihasrc/Music/AutoX-SemMap-main/roomwatch/perception/ckpts/yoloworld/yoloworld_house.onnx",
    imgsz=640,
):
    if classes is None:
        classes = ["table", "chair", "sofa", "bed", "cabinet", "refrigerator", "door"]

    out_dir = os.path.dirname(os.path.abspath(output_onnx))
    os.makedirs(out_dir, exist_ok=True)

    print(f"Loading base model {model_name}...")
    model = YOLOWorld(model_name)

    print(f"Setting offline vocabulary: {classes}")
    model.set_classes(classes)

    custom_pt = os.path.join(out_dir, "yoloworld_house.pt")
    model.save(custom_pt)
    print(f"Saved re-parameterized PyTorch model to: {custom_pt}")

    print(f"Exporting to ONNX format with image size {imgsz}...")
    model.export(format="onnx", imgsz=imgsz, dynamic=False)
    
    exported_onnx = os.path.join(out_dir, "yoloworld_house.onnx")
    if os.path.exists(exported_onnx):
        print(f"SUCCESS: Exported YOLO-World ONNX model to: {exported_onnx}")
    else:
        print(f"WARNING: Checking directory: {os.listdir(out_dir)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--classes", nargs="+", default=["table", "chair", "sofa", "bed", "cabinet", "refrigerator", "door"])
    parser.add_argument("--model", default="yolov8s-worldv2.pt")
    parser.add_argument("--output", default="/home/srihasrc/Music/AutoX-SemMap-main/roomwatch/perception/ckpts/yoloworld/yoloworld_house.onnx")
    args = parser.parse_args()
    export_yoloworld_onnx(classes=args.classes, model_name=args.model, output_onnx=args.output)
