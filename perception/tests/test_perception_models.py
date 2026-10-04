"""Test perception predictors on CPU."""

import numpy as np
from PIL import Image as PILImg
import torch
from roomwatch_perception.perception import (
    FakeObjectPredictor,
    FakeSAMPredictor,
    ObjectPredictor,
)


def test_bbox_to_scaled_xyxy():
    pred = ObjectPredictor(device="cpu")
    # center at (0.5, 0.5), w=0.2, h=0.4 in 100x100 image
    bboxes = torch.tensor([[0.5, 0.5, 0.2, 0.4]])
    xyxy = pred.bbox_to_scaled_xyxy(bboxes, 100, 100)
    assert xyxy.shape == (1, 4)
    # x1=40, y1=30, x2=60, y2=70
    assert torch.allclose(xyxy, torch.tensor([[40.0, 30.0, 60.0, 70.0]]))


def test_fake_predictors_on_synthetic_image():
    # Create image with red (door), green (chair), blue (table) blocks like mock robot
    img_np = np.full((240, 320, 3), 120, dtype=np.uint8)
    img_np[90:200, 40:100] = [200, 40, 40]   # red -> door
    img_np[90:200, 130:190] = [40, 200, 40]  # green -> chair
    img_np[90:200, 220:280] = [40, 40, 200]  # blue -> table

    img_pil = PILImg.fromarray(img_np)
    fake_det = FakeObjectPredictor(device="cpu")
    bboxes, phrases, conf = fake_det.predict(img_pil, det_text_prompt="table . door . chair .")

    assert len(phrases) == 3
    assert set(phrases) == {"door", "chair", "table"}
    assert bboxes.shape == (3, 4)

    # Convert to xyxy and run fake SAM
    xyxy = fake_det.bbox_to_scaled_xyxy(bboxes, 320, 240)
    fake_sam = FakeSAMPredictor(device="cpu")
    _, masks = fake_sam.predict(img_pil, xyxy)
    assert masks.shape == (3, 1, 240, 320)
