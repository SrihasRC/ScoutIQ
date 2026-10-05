"""Perception models for object detection and segmentation (CPU-only)."""

import logging
import os
import warnings
from typing import List, Optional, Tuple, Union

import numpy as np
from PIL import Image as PILImg
import torch
import cv2
from torchvision.ops import box_convert

# Filter noisy warnings on CPU
warnings.filterwarnings("ignore", message=".*Failed to load custom C.*")
warnings.filterwarnings("ignore", message=".*torch.meshgrid.*")
warnings.filterwarnings("ignore", message=".*Importing from timm.*")
warnings.filterwarnings("ignore", message=".*Overwriting.*in registry.*")
warnings.filterwarnings("ignore", message=".*resume_download.*")

from groundingdino.models import build_model
import groundingdino.datasets.transforms as T
from groundingdino.util.inference import predict
from groundingdino.util.slconfig import SLConfig
from groundingdino.util.utils import clean_state_dict
from mobile_sam import SamAutomaticMaskGenerator, SamPredictor, sam_model_registry


def find_checkpoint_path(subpath: str) -> Optional[str]:
    """Finds checkpoint file in known paths or environment variable."""
    env_dir = os.environ.get("ROOMWATCH_CKPTS_DIR")
    if env_dir:
        cand = os.path.join(env_dir, subpath)
        if os.path.exists(cand):
            return os.path.abspath(cand)

    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates = [
        os.path.join(pkg_root, "ckpts", subpath),
        os.path.join(pkg_root, "..", "ckpts", subpath),
        os.path.join(pkg_root, "..", "perception", "ckpts", subpath),
        os.path.join(os.getcwd(), "perception", "ckpts", subpath),
        os.path.join(os.getcwd(), "ckpts", subpath),
    ]
    for cand in candidates:
        if os.path.exists(cand):
            return os.path.abspath(cand)
    return None


class Logger(object):
    def __init__(self):
        super(Logger, self).__init__()
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)


class Device(object):
    """Device class, explicitly configured for CPU execution."""
    def __init__(self, device: str = "cpu"):
        super(Device, self).__init__()
        self.device = "cpu"
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)


class CommonContextObject(Logger, Device):
    def __init__(self, device: str = "cpu"):
        super(CommonContextObject, self).__init__()
        self.device = device


class ObjectPredictor(CommonContextObject):
    """Root class for object prediction."""
    def __init__(self, device: str = "cpu"):
        super(ObjectPredictor, self).__init__(device=device)

    def bbox_to_scaled_xyxy(self, bboxes: torch.Tensor, img_w: int, img_h: int) -> torch.Tensor:
        """Converts normalized cxcywh bounding boxes to scaled xyxy format."""
        if len(bboxes) == 0:
            return torch.empty((0, 4), dtype=torch.float32)
        scale = torch.tensor([img_w, img_h, img_w, img_h], dtype=torch.float32, device=bboxes.device)
        scaled_boxes = bboxes * scale
        return box_convert(boxes=scaled_boxes, in_fmt="cxcywh", out_fmt="xyxy")


class YOLOWorldONNXPredictor(ObjectPredictor):
    """
    High-performance zero-shot / open-vocabulary object detector using YOLO-World ONNX on CPU.
    Bakes offline vocabulary embeddings for instant real-time CPU execution.
    """
    def __init__(
        self,
        onnx_path: Optional[str] = None,
        vocab_path: Optional[str] = None,
        device: str = "cpu"
    ):
        super(YOLOWorldONNXPredictor, self).__init__(device="cpu")
        import onnxruntime as ort

        # 1. Resolve ONNX model
        if onnx_path is None:
            onnx_path = os.environ.get("YOLOWORLD_ONNX_PATH")
        if onnx_path is None or not os.path.exists(onnx_path):
            onnx_path = find_checkpoint_path("yoloworld/yoloworld_house.onnx")
        if onnx_path is None or not os.path.exists(onnx_path):
            onnx_path = find_checkpoint_path("yoloworld/yolov8s-worldv2.onnx")

        if onnx_path is None or not os.path.exists(onnx_path):
            raise FileNotFoundError(
                f"YOLO-World ONNX model not found in perception/ckpts/yoloworld/"
            )

        self.logger.info("Loading YOLO-World ONNX model from %s", onnx_path)
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = 4
        sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        self.session = ort.InferenceSession(
            onnx_path,
            sess_options=sess_options,
            providers=["CPUExecutionProvider"]
        )

        self.input_names = [inp.name for inp in self.session.get_inputs()]
        self.output_names = [out.name for out in self.session.get_outputs()]

        # 2. Resolve offline vocabulary text features
        if vocab_path is None:
            vocab_path = os.environ.get("YOLOWORLD_VOCAB_PATH")
        if vocab_path is None or not os.path.exists(vocab_path):
            vocab_path = find_checkpoint_path("yoloworld/house_classes_txt_feats.npz")
        if vocab_path is None or not os.path.exists(vocab_path):
            vocab_path = find_checkpoint_path("yoloworld/coco80_txt_feats.npz")

        if vocab_path is not None and os.path.exists(vocab_path):
            self.logger.info("Loading offline text embeddings from %s", vocab_path)
            data = np.load(vocab_path, allow_pickle=True)
            self.txt_feats = data["txt_feats"]
            raw_classes = data["classes"].tolist() if hasattr(data["classes"], "tolist") else list(data["classes"])
            self.classes = [str(c).lower().strip() for c in raw_classes]
        else:
            self.logger.warn("No offline vocabulary found; initializing dummy embeddings.")
            self.txt_feats = np.zeros((1, 80, 512), dtype=np.float32)
            self.classes = [f"class_{i}" for i in range(80)]

        # Map common synonyms to standard indoor map classes
        self.synonym_map = {
            "dining table": "table",
            "couch": "sofa",
            "coffee table": "table",
            "desk": "table",
        }

    def _letterbox(self, img_pil: PILImg.Image, target_size: int = 640):
        orig_w, orig_h = img_pil.size
        scale = min(target_size / orig_w, target_size / orig_h)
        nw, nh = int(round(orig_w * scale)), int(round(orig_h * scale))
        resized = img_pil.resize((nw, nh), PILImg.BILINEAR)

        canvas = PILImg.new("RGB", (target_size, target_size), (114, 114, 114))
        pad_x = (target_size - nw) // 2
        pad_y = (target_size - nh) // 2
        canvas.paste(resized, (pad_x, pad_y))

        arr = np.array(canvas, dtype=np.float32) / 255.0
        tensor = np.transpose(arr, (2, 0, 1))[np.newaxis, ...]  # (1, 3, 640, 640)
        return tensor, scale, pad_x, pad_y, orig_w, orig_h

    def predict(
        self,
        image_pil: PILImg.Image,
        det_text_prompt: str = "table . chair . sofa . bed . cabinet . refrigerator . door .",
        box_threshold: float = 0.35,
        text_threshold: float = 0.35,
        target_size: int = 640
    ) -> Tuple[torch.Tensor, List[str], torch.Tensor]:
        tensor, scale, pad_x, pad_y, orig_w, orig_h = self._letterbox(image_pil, target_size=target_size)

        feed_dict = {self.input_names[0]: tensor}
        if len(self.input_names) > 1 and "txt_feats" in self.input_names[1]:
            feed_dict[self.input_names[1]] = self.txt_feats

        outputs = self.session.run(self.output_names, feed_dict)
        preds = outputs[0]  # shape: (1, 4 + C, 8400) or (1, 8400, 4 + C)
        if preds.shape[1] < preds.shape[2]:
            preds = np.transpose(preds[0], (1, 0))  # (8400, 4 + C)
        else:
            preds = preds[0]

        boxes_xywh = preds[:, :4]  # cx, cy, w, h in 640x640 space
        scores = preds[:, 4:]

        # Apply sigmoid if logits
        if np.min(scores) < 0.0 or np.max(scores) > 1.0:
            scores = 1.0 / (1.0 + np.exp(-np.clip(scores, -15.0, 15.0)))

        max_scores = np.max(scores, axis=-1)
        best_classes = np.argmax(scores, axis=-1)

        mask = max_scores >= box_threshold
        if not np.any(mask):
            return torch.empty((0, 4), dtype=torch.float32), [], torch.empty((0,), dtype=torch.float32)

        cand_boxes = boxes_xywh[mask]
        cand_scores = max_scores[mask]
        cand_cids = best_classes[mask]

        # Parse requested prompt tokens
        prompt_tokens = [tok.strip().lower() for tok in det_text_prompt.split(".") if tok.strip()]

        out_boxes_norm = []
        out_boxes_pixels = []
        out_scores = []
        out_phrases = []

        for box, score, cid in zip(cand_boxes, cand_scores, cand_cids):
            raw_cat = self.classes[cid] if cid < len(self.classes) else f"object_{cid}"
            cat = self.synonym_map.get(raw_cat, raw_cat)

            # Filter against active prompt classes
            if prompt_tokens:
                match = any(token == cat or token in cat or cat in token for token in prompt_tokens)
                if not match:
                    continue

            cx_canv, cy_canv, bw_canv, bh_canv = box
            # Undo padding and unscale
            cx_orig = (cx_canv - pad_x) / scale
            cy_orig = (cy_canv - pad_y) / scale
            bw_orig = bw_canv / scale
            bh_orig = bh_canv / scale

            # Normalized cx, cy, w, h in [0, 1]
            cx_norm = np.clip(cx_orig / orig_w, 0.0, 1.0)
            cy_norm = np.clip(cy_orig / orig_h, 0.0, 1.0)
            bw_norm = np.clip(bw_orig / orig_w, 0.0, 1.0)
            bh_norm = np.clip(bh_orig / orig_h, 0.0, 1.0)

            # Pixel xywh for cv2.dnn.NMSBoxes
            x1_px = int((cx_orig - bw_orig / 2.0))
            y1_px = int((cy_orig - bh_orig / 2.0))
            w_px = int(bw_orig)
            h_px = int(bh_orig)

            out_boxes_norm.append([cx_norm, cy_norm, bw_norm, bh_norm])
            out_boxes_pixels.append([x1_px, y1_px, w_px, h_px])
            out_scores.append(float(score))
            out_phrases.append(cat)

        if not out_boxes_norm:
            return torch.empty((0, 4), dtype=torch.float32), [], torch.empty((0,), dtype=torch.float32)

        indices = cv2.dnn.NMSBoxes(
            bboxes=out_boxes_pixels,
            scores=out_scores,
            score_threshold=box_threshold,
            nms_threshold=0.45,
        )

        if len(indices) == 0:
            return torch.empty((0, 4), dtype=torch.float32), [], torch.empty((0,), dtype=torch.float32)

        indices = np.array(indices).flatten()
        final_boxes = [out_boxes_norm[i] for i in indices]
        final_scores = [out_scores[i] for i in indices]
        final_phrases = [out_phrases[i] for i in indices]

        return (
            torch.tensor(final_boxes, dtype=torch.float32),
            final_phrases,
            torch.tensor(final_scores, dtype=torch.float32)
        )


class GroundingDINOObjectPredictor(ObjectPredictor):
    """Zero-shot object detection using GroundingDINO on CPU."""
    def __init__(
        self,
        config_path: Optional[str] = None,
        ckpt_path: Optional[str] = None,
        device: str = "cpu"
    ):
        super(GroundingDINOObjectPredictor, self).__init__(device="cpu")
        self.device = "cpu"

        # Resolve config file
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "cfg", "gdino", "GroundingDINO_SwinT_OGC.py"
            )
        self.config_file = config_path

        # Resolve checkpoint
        if ckpt_path is None:
            ckpt_path = os.environ.get("GDINO_CKPT_PATH")
        if ckpt_path is None or not os.path.exists(ckpt_path):
            ckpt_path = find_checkpoint_path("gdino/gdino.pth")

        self.ckpt_path = ckpt_path
        self.ckpt_repo_id = "ShilongLiu/GroundingDINO"
        self.ckpt_filename = "groundingdino_swint_ogc.pth"

        self.model = self._load_model()

    def _load_model(self) -> torch.nn.Module:
        args = SLConfig.fromfile(self.config_file)
        args.device = self.device
        model = build_model(args)

        if self.ckpt_path and os.path.exists(self.ckpt_path):
            self.logger.info("Loading GroundingDINO from local checkpoint: %s", self.ckpt_path)
            checkpoint = torch.load(self.ckpt_path, map_location=self.device)
        else:
            self.logger.info("Downloading GroundingDINO from HuggingFace hub...")
            from huggingface_hub import hf_hub_download
            cache_file = hf_hub_download(repo_id=self.ckpt_repo_id, filename=self.ckpt_filename)
            checkpoint = torch.load(cache_file, map_location=self.device)

        model.load_state_dict(clean_state_dict(checkpoint["model"]), strict=False)
        model.eval()
        return model

    def image_transform_grounding(
        self,
        image_pil: PILImg.Image,
        target_size: int = 800
    ) -> Tuple[PILImg.Image, torch.Tensor]:
        transform = T.Compose([
            T.RandomResize([target_size], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])
        image_tensor, _ = transform(image_pil, None)
        return image_pil, image_tensor

    def predict(
        self,
        image_pil: PILImg.Image,
        det_text_prompt: str = "objects",
        box_threshold: float = 0.35,
        text_threshold: float = 0.35,
        target_size: int = 800
    ) -> Tuple[torch.Tensor, List[str], torch.Tensor]:
        """Runs GroundingDINO prediction on CPU."""
        _, image_tensor = self.image_transform_grounding(image_pil, target_size=target_size)
        bboxes, conf, phrases = predict(
            self.model,
            image_tensor,
            det_text_prompt,
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            device=self.device
        )
        return bboxes, phrases, conf


class SegmentAnythingPredictor(ObjectPredictor):
    """Segmentation predictor using MobileSAM on CPU."""
    def __init__(self, checkpoint_path: Optional[str] = None, device: str = "cpu"):
        super(SegmentAnythingPredictor, self).__init__(device="cpu")
        self.device = "cpu"

        if checkpoint_path is None:
            checkpoint_path = os.environ.get("MOBILESAM_CKPT_PATH")
        if checkpoint_path is None or not os.path.exists(checkpoint_path):
            checkpoint_path = find_checkpoint_path("mobilesam/vit_t.pth")

        if checkpoint_path is None or not os.path.exists(checkpoint_path):
            raise FileNotFoundError(
                f"MobileSAM checkpoint not found. Looked in perception/ckpts/mobilesam/vit_t.pth"
            )

        self.logger.info("Loading MobileSAM from %s", checkpoint_path)
        self.sam = sam_model_registry["vit_t"](checkpoint=checkpoint_path)
        self.sam.to(device=self.device)
        self.sam.eval()
        self.predictor = SamPredictor(self.sam)
        self.mask_generator = SamAutomaticMaskGenerator(self.sam)

    def predict(
        self,
        image: Union[PILImg.Image, np.ndarray],
        prompt_bboxes: Optional[Union[torch.Tensor, List, np.ndarray]]
    ) -> Tuple[Optional[torch.Tensor], Optional[torch.Tensor]]:
        """Predicts masks given prompt bounding boxes."""
        image_np = np.array(image)
        h, w = image_np.shape[:2]

        if prompt_bboxes is not None and len(prompt_bboxes) > 0:
            if not isinstance(prompt_bboxes, torch.Tensor):
                input_boxes = torch.tensor(prompt_bboxes, dtype=torch.float32, device=self.device)
            else:
                input_boxes = prompt_bboxes.to(device=self.device, dtype=torch.float32)

            self.predictor.set_image(image_np)
            transformed_boxes = self.predictor.transform.apply_boxes_torch(input_boxes, (h, w))
            masks, _, _ = self.predictor.predict_torch(
                point_coords=None,
                point_labels=None,
                boxes=transformed_boxes,
                multimask_output=False,
            )
            return input_boxes, masks
        elif prompt_bboxes is not None and len(prompt_bboxes) == 0:
            return torch.empty((0, 4), device=self.device), torch.empty((0, 1, h, w), dtype=torch.bool, device=self.device)
        else:
            masks = self.mask_generator.generate(image_np)
            return None, masks


class FakeObjectPredictor(ObjectPredictor):
    """
    Mock detector for rapid unit testing and simulator-free testing.
    Can detect synthetic colored blocks in mock robot camera images:
    - Red block (col ~ [200, 40, 40]) -> door
    - Green block (col ~ [40, 200, 40]) -> chair
    - Blue block (col ~ [40, 40, 200]) -> table
    """
    def __init__(self, device: str = "cpu"):
        super(FakeObjectPredictor, self).__init__(device=device)

    def predict(
        self,
        image_pil: PILImg.Image,
        det_text_prompt: str = "table . door . chair .",
        box_threshold: float = 0.35,
        text_threshold: float = 0.35,
        target_size: int = 800
    ) -> Tuple[torch.Tensor, List[str], torch.Tensor]:
        im = np.array(image_pil.convert("RGB"))
        h, w = im.shape[:2]

        bboxes_list = []
        conf_list = []
        phrases_list = []

        # Color segmentation for mock_robot blocks:
        # Mock robot blocks are at u in [40, 100], [130, 190], [220, 280], v in [90, 200]
        # Red: R > 150 and G < 100 and B < 100
        red_mask = (im[:, :, 0] > 150) & (im[:, :, 1] < 100) & (im[:, :, 2] < 100)
        # Green: G > 150 and R < 100 and B < 100
        green_mask = (im[:, :, 1] > 150) & (im[:, :, 0] < 100) & (im[:, :, 2] < 100)
        # Blue: B > 150 and R < 100 and G < 100
        blue_mask = (im[:, :, 2] > 150) & (im[:, :, 0] < 100) & (im[:, :, 1] < 100)

        for mask, cat in [(red_mask, "door"), (green_mask, "chair"), (blue_mask, "table")]:
            if np.any(mask) and cat in det_text_prompt:
                y_idxs, x_idxs = np.where(mask)
                x0, x1 = float(np.min(x_idxs)), float(np.max(x_idxs))
                y0, y1 = float(np.min(y_idxs)), float(np.max(y_idxs))
                cx = (x0 + x1) / 2.0 / w
                cy = (y0 + y1) / 2.0 / h
                bw = (x1 - x0) / w
                bh = (y1 - y0) / h
                bboxes_list.append([cx, cy, bw, bh])
                conf_list.append(0.85)
                phrases_list.append(cat)

        if not bboxes_list:
            # Default fallback detection so test always succeeds if needed
            return torch.empty((0, 4), dtype=torch.float32), [], torch.empty((0,), dtype=torch.float32)

        return (
            torch.tensor(bboxes_list, dtype=torch.float32),
            phrases_list,
            torch.tensor(conf_list, dtype=torch.float32)
        )


class FakeSAMPredictor(ObjectPredictor):
    """Fast mock SAM predictor generating bounding box masks."""
    def __init__(self, device: str = "cpu"):
        super(FakeSAMPredictor, self).__init__(device=device)

    def predict(
        self,
        image: Union[PILImg.Image, np.ndarray],
        prompt_bboxes: Optional[Union[torch.Tensor, List, np.ndarray]]
    ) -> Tuple[Optional[torch.Tensor], Optional[torch.Tensor]]:
        im = np.array(image)
        h, w = im.shape[:2]
        if prompt_bboxes is None or len(prompt_bboxes) == 0:
            return torch.empty((0, 4)), torch.empty((0, 1, h, w), dtype=torch.bool)

        if not isinstance(prompt_bboxes, torch.Tensor):
            boxes = torch.tensor(prompt_bboxes, dtype=torch.float32)
        else:
            boxes = prompt_bboxes

        num_boxes = len(boxes)
        masks = torch.zeros((num_boxes, 1, h, w), dtype=torch.bool)
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
            x1, x2 = max(0, x1), min(w, x2)
            y1, y2 = max(0, y1), min(h, y2)
            masks[i, 0, y1:y2, x1:x2] = True

        return boxes, masks


class DepthPredictor(CommonContextObject):
    def __init__(self, device: str = "cpu"):
        super(DepthPredictor, self).__init__(device=device)

    def predict(self, img_pil):
        raise NotImplementedError("predict method must be implemented by subclasses")


class DepthAnythingPredictor(DepthPredictor):
    """Lazy-loaded depth predictor."""
    def __init__(self, device: str = "cpu"):
        super(DepthAnythingPredictor, self).__init__(device=device)
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        self.image_processor = AutoImageProcessor.from_pretrained("LiheYoung/depth-anything-small-hf")
        self.model = AutoModelForDepthEstimation.from_pretrained("LiheYoung/depth-anything-small-hf")

    def predict(self, img_pil):
        image = img_pil.convert("RGB")
        inputs = self.image_processor(images=image, return_tensors="pt")
        with torch.no_grad():
            outputs = self.model(**inputs)
            predicted_depth = outputs.predicted_depth

        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1),
            size=image.size[::-1],
            mode="bicubic",
            align_corners=False,
        )
        output = prediction.squeeze().cpu().numpy()
        formatted = (output * 255.0 / np.max(output)).astype("uint8")
        depth_pil = PILImg.fromarray(formatted)
        return depth_pil, output
