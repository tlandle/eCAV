import time
from collections import deque

import erdos

from pylot.perception.messages import ObstaclesMessage


class ObjectTrackerManager():
    def __init__(self, tracker_type, flags):
        self._tracker_type = tracker_type
        # Absolute time when the last tracker run completed.
        self._last_tracker_run_completion_time = 0
        try:
            if tracker_type == 'da_siam_rpn':
                from pylot.perception.tracking.da_siam_rpn_tracker import\
                    MultiObjectDaSiamRPNTracker
                self._tracker = MultiObjectDaSiamRPNTracker(
                    self._flags, self._logger)
            elif tracker_type == 'deep_sort':
                from pylot.perception.tracking.deep_sort_tracker import\
                    MultiObjectDeepSORTTracker
                self._tracker = MultiObjectDeepSORTTracker(
                    self._flags, self._logger)
            elif tracker_type == 'sort':
                from pylot.perception.tracking.sort_tracker import\
                    MultiObjectSORTTracker
                self._tracker = MultiObjectSORTTracker(self._flags,
                                                       self._logger)
            else:
                raise ValueError(
                    'Unexpected tracker type {}'.format(tracker_type))
        except ImportError as error:
            self._logger.fatal('Error importing {}'.format(tracker_type))
            raise error

        self._obstacles_msgs = deque()
        self._frame_msgs = deque()
        self._detection_update_count = -1

    def _run_tracker(self, camera_frame):
        start = time.time()
        result = self._tracker.track(camera_frame)
        return (time.time() - start) * 1000, result

    def track_obstacles(self, timestamp, camera_frame):
        tracked_obstacles = []
        tracker_runtime, (ok, tracked_obstacles) = \
            self._run_tracker(camera_frame)

        return tracked_obstacles

    def __compute_tracker_delay(self, world_time, detector_runtime,
                                tracker_runtime):
        # If the tracker runtime does not fit within the frame gap, then
        # the tracker will fall behind. We need a scheduler to better
        # handle such situations.
        if (world_time + detector_runtime >
                self._last_tracker_run_completion_time):
            # The detector finished after the previous tracker invocation
            # completed. Therefore, the tracker is already sequenced.
            tracker_runtime = detector_runtime + tracker_runtime
            self._last_tracker_run_completion_time = \
                world_time + tracker_runtime
        else:
            # The detector finished before the previous tracker invocation
            # completed. The tracker can only run after the previous
            # invocation completes.
            self._last_tracker_run_completion_time += tracker_runtime
            tracker_runtime = \
                self._last_tracker_run_completion_time - world_time
        return tracker_runtime
