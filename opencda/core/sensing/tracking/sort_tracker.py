import numpy as np

#from pylot.perception.detection.obstacle import Obstacle
#from pylot.perception.detection.utils import BoundingBox2D
from opencda.core.sensing.perception.obstacle_vehicle import BoundingBox
from opencda.core.sensing.tracking.multi_object_tracker import MultiObjectTracker

from sort.sort import Sort
from collections import deque


import logging
logger = logging.getLogger(__name__)

class MultiObjectSORTTracker(MultiObjectTracker):
    def __init__(self, max_age, min_matching_iou):
        """ Initializes the SORT tracker.
        Args:
            max_age (int): Maximum number of frames to keep a track alive.
            min_matching_iou (float): Minimum IOU to consider a match.
            logger: Logger to log messages.
        """
        self.tracker = Sort(max_age=max_age,
                            min_hits=1,
                            min_iou=min_matching_iou)
        self.history = deque(maxlen=10)

    def track(self, detections):
        """ Tracks obstacles in a frame.
        Args:
            frame: Current frame.
            detections: List of detected obstacles.
        Returns:
            bool: True if tracking was successful.
            list: List of tracked obstacles.

        """
        # each track in tracks has format ([xmin, ymin, xmax, ymax], id)
        convert_detections, labels, ids = self.convert_detections(detections)
        tracks = self.tracker.update(convert_detections,
                                      labels=labels,
                                      ids=ids)

        logger.debug('tracks: %s' %tracks)
        #logger.debug('tracks:', tracks)
        self.history.append(tracks.copy()) # for input into predictor
        return tracks.copy()

    def convert_detections(self, detections):
        # Prepare detections for SORT

        if detections.is_cuda:
            detections = detections.cpu().detach().numpy()
        else:
            detections = detections.detach().numpy()
        dets_for_sort = []
        labels = []
        ids = []
        #logger.debug('detections:', detections)
        for det in detections:
            # yolo bbx format: [x1, y1, x2, y2, conf, cls]
            x1, y1, x2, y2, conf, cls = det
            # Optionally filter classes (e.g., only cars/trucks)
            dets_for_sort.append([x1.item(), y1.item(), x2.item(), y2.item(), conf.item()])
            labels.append(cls.item())
            ids.append(-1)
        #logger.debug('dets_for_sort:', dets_for_sort)
        return (np.array(dets_for_sort), labels, ids)

    def convert_detections_for_sort_alg(self, obstacles):
        converted_detections = []
        labels = []
        ids = []
        for obstacle in obstacles:
            bbox = [
                obstacle.bounding_box_2D.x_min, obstacle.bounding_box_2D.y_min,
                obstacle.bounding_box_2D.x_max, obstacle.bounding_box_2D.y_max,
                obstacle.confidence
            ]
            converted_detections.append(bbox)
            labels.append(obstacle.label)
            ids.append(obstacle.id)
        return (np.array(converted_detections), labels, ids)
