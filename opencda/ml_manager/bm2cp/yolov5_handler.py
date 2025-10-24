# yolov5_handler.py
import base64
import json
import io
import cv2
import torch
import numpy as np
from ts.torch_handler.base_handler import BaseHandler

class Yolov5Handler(BaseHandler):
    """
    TorchServe handler for YOLOv5.  Accepts a JSON object containing
    a base64-encoded BGR image under the key 'image'.
    Returns a list of detection dictionaries with bbox, score and label.
    """

    def initialize(self, context):
        """
        Load YOLOv5 model from torch.hub (or from a local path).
        """
        # `force_reload=True` if you want to bypass the hub cache
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True).eval()
        # Warm up the model so first request is fast
        self.model(torch.zeros((1, 3, 640, 640)))

    def preprocess(self, requests):
        """
        Decode the base64 image strings into OpenCV images (BGR format).
        TorchServe passes each request in a list; return a list of images.
        """
        images = []
        for req in requests:
            data = req.get('body')
            if isinstance(data, (bytes, bytearray)):
                # TorchServe may deliver raw bytes; decode JSON first
                data = json.loads(data.decode('utf-8'))
            img_bytes = base64.b64decode(data['image'])
            img_array = np.frombuffer(img_bytes, np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            images.append(img)
        return images

    def inference(self, images):
        """
        Perform YOLO inference on the list of BGR images.
        Returns a list of lists of detection dicts.
        """
        results = []
        for img in images:
            pred = self.model(img)  # inference
            det = pred.xyxy[0]      # tensor of shape [N,6]: x1,y1,x2,y2,confidence,class_id
            boxes = []
            for *xyxy, conf, cls_id in det.cpu().numpy():
                x1, y1, x2, y2 = xyxy
                label = pred.names[int(cls_id)]
                boxes.append({
                    "bbox": [float(x1), float(y1), float(x2), float(y2)],
                    "score": float(conf),
                    "label": label
                })
            results.append(boxes)
        return results

    def postprocess(self, results):
        """
        TorchServe expects a list of responses; just return the inference result.
        """
        return results
