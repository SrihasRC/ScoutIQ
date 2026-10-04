"""Standalone demo script providing CPU timing report for perception pipeline."""

import argparse
import os
import sys
import time

import numpy as np
from PIL import Image as PILImg

from .perception import GroundingDINOObjectPredictor, SegmentAnythingPredictor
from .utils import annotate, filter_large_boxes, overlay_masks


def run_demo(
    image_path: str,
    prompt: str = "table . door . chair .",
    box_threshold: float = 0.35,
    text_threshold: float = 0.35,
    output_path: str = "demo_output.jpg",
    target_size: int = 800,
) -> None:
    if not os.path.exists(image_path):
        print(f"Error: Image not found: {image_path}")
        sys.exit(1)

    print("=" * 60)
    print("RoomWatch Perception CPU Demo & Timing Report")
    print("=" * 60)
    print(f"Input image: {image_path}")
    print(f"Text prompt: '{prompt}'")
    print(f"Box threshold: {box_threshold}, Text threshold: {text_threshold}")
    print(f"Target size: {target_size}")
    print("Device: CPU")
    print("-" * 60)

    # 1. Model initialization
    t0 = time.time()
    print("Loading GroundingDINO (CPU)...")
    gdino = GroundingDINOObjectPredictor(device="cpu")
    t_gdino_init = time.time() - t0
    print(f"GroundingDINO loaded in {t_gdino_init:.2f}s")

    t0 = time.time()
    print("Loading MobileSAM (CPU)...")
    sam = SegmentAnythingPredictor(device="cpu")
    t_sam_init = time.time() - t0
    print(f"MobileSAM loaded in {t_sam_init:.2f}s")

    # Load image
    img_pil = PILImg.open(image_path).convert("RGB")
    w, h = img_pil.size
    print(f"Image dimensions: {w}x{h}")
    print("-" * 60)

    # 2. GroundingDINO Inference
    print("Running GroundingDINO detection...")
    t0 = time.time()
    bboxes, phrases, conf = gdino.predict(
        img_pil,
        det_text_prompt=prompt,
        box_threshold=box_threshold,
        text_threshold=text_threshold,
        target_size=target_size,
    )
    t_gdino_infer = time.time() - t0
    print(f"GroundingDINO inference time: {t_gdino_infer:.2f}s")
    print(f"Detected {len(phrases)} object(s):")
    for phrase, score in zip(phrases, conf):
        print(f"  - {phrase}: {float(score):.2f}")

    # 3. MobileSAM Inference
    if len(bboxes) > 0:
        scaled_bboxes = gdino.bbox_to_scaled_xyxy(bboxes, w, h)
        print("Running MobileSAM segmentation...")
        t0 = time.time()
        scaled_bboxes, masks = sam.predict(img_pil, scaled_bboxes)
        t_sam_infer = time.time() - t0
        print(f"MobileSAM inference time: {t_sam_infer:.2f}s")

        # Filter large boxes
        scaled_bboxes, index = filter_large_boxes(scaled_bboxes, w, h, threshold=0.5)
        masks = masks[index]
        conf = conf[index]
        ind = np.where(index)[0]
        phrases = [phrases[i] for i in ind]

        # 4. Annotation
        t0 = time.time()
        annotated_pil = annotate(overlay_masks(img_pil, masks), scaled_bboxes, conf, phrases)
        t_annotate = time.time() - t0
    else:
        t_sam_infer = 0.0
        t_annotate = 0.0
        annotated_pil = img_pil

    annotated_pil.save(output_path)
    print(f"Saved visualization to: {output_path}")

    # Timing Summary Report
    print("=" * 60)
    print("PERCEPTION PIPELINE TIMING REPORT (CPU)")
    print("=" * 60)
    print(f"GroundingDINO Model Load:   {t_gdino_init:6.2f} s")
    print(f"MobileSAM Model Load:       {t_sam_init:6.2f} s")
    print(f"GroundingDINO Inference:    {t_gdino_infer:6.2f} s")
    print(f"MobileSAM Inference:        {t_sam_infer:6.2f} s")
    print(f"Mask Overlay & Annotation:  {t_annotate:6.2f} s")
    total_infer = t_gdino_infer + t_sam_infer + t_annotate
    print(f"Total Per-Frame Inference:  {total_infer:6.2f} s ({1.0 / max(total_infer, 1e-4):.3f} FPS)")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="RoomWatch Perception Demo on CPU.")
    parser.add_argument("image", help="Path to input image")
    parser.add_argument("prompt", nargs="?", default="table . door . chair .", help="Text prompt")
    parser.add_argument("--box-threshold", type=float, default=0.35, help="Box threshold")
    parser.add_argument("--text-threshold", type=float, default=0.35, help="Text threshold")
    parser.add_argument("--output", "-o", default="demo_output.jpg", help="Output annotated image path")
    parser.add_argument("--target-size", type=int, default=800, help="Target image size for detection")

    args = parser.parse_args()
    run_demo(
        image_path=args.image,
        prompt=args.prompt,
        box_threshold=args.box_threshold,
        text_threshold=args.text_threshold,
        output_path=args.output,
        target_size=args.target_size,
    )


if __name__ == "__main__":
    main()
