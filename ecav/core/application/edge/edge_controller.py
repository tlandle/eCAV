"""Runtime controller for adaptive edge pipeline.

Implements Paper 2's three controller knobs:
1. Predictor tier: heavy (SMART) vs light (linear)
2. Fusion fidelity: full vs reduced vs skip
3. Update rate: process vs skip frame

The controller monitors queue depth and per-stage latency to decide
what to do each tick. Under low load, it uses the heavy predictor
and full fusion. Under high load, it degrades gracefully.
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import logging
import time
from collections import deque
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class EdgeController:
    """Adaptive controller for the composable edge pipeline.

    Policies:
        static_heavy: Always use heavy predictor, full fusion.
        static_light: Always use light predictor, full fusion.
        adaptive:     Switch based on queue depth and deadline pressure.
    """

    def __init__(self, cfg: dict):
        self.policy = cfg.get('policy', 'static_heavy')
        self.deadline_ms = cfg.get('deadline_ms', 200.0)
        self.queue_depth_thresh = cfg.get('queue_depth_thresh', 3)
        self.metrics_window = cfg.get('metrics_window', 10)

        # Rolling metrics
        self._stage_times: deque = deque(maxlen=self.metrics_window)
        self._queue_depths: deque = deque(maxlen=self.metrics_window)

        # Last decisions (for logging/metrics)
        self.last_predictor_tier = 'heavy'
        self.last_fusion_fidelity = 'full'
        self.last_skipped = False

    def record_tick(self, queue_depth: int, stage_latencies: Dict[str, float]):
        """Record metrics from the latest tick for adaptive decisions."""
        self._queue_depths.append(queue_depth)
        total_ms = sum(stage_latencies.values()) * 1000
        self._stage_times.append(total_ms)

    def select_predictor(self, queue_depth: int,
                         stage_latencies: Dict[str, float],
                         num_agents: int) -> str:
        """Select predictor tier for this tick.

        Returns:
            'heavy' or 'light' key into the predictors dict.
        """
        if self.policy == 'static_heavy':
            self.last_predictor_tier = 'heavy'
            return 'heavy'
        elif self.policy == 'static_light':
            self.last_predictor_tier = 'light'
            return 'light'

        # Adaptive: switch to light when overloaded
        avg_time = (sum(self._stage_times) / len(self._stage_times)
                    if self._stage_times else 0)
        avg_queue = (sum(self._queue_depths) / len(self._queue_depths)
                     if self._queue_depths else 0)

        if avg_time > self.deadline_ms * 0.8 or avg_queue > self.queue_depth_thresh:
            self.last_predictor_tier = 'light'
            return 'light'

        self.last_predictor_tier = 'heavy'
        return 'heavy'

    def should_skip_frame(self, queue_depth: int) -> bool:
        """Update rate knob: skip fusion when behind."""
        if self.policy in ('static_heavy', 'static_light'):
            self.last_skipped = False
            return False

        # Adaptive: skip if queue is deep (behind by multiple frames)
        skip = queue_depth > self.queue_depth_thresh * 2
        self.last_skipped = skip
        if skip:
            logger.info("[CONTROLLER] Skipping frame (queue_depth=%d)",
                        queue_depth)
        return skip

    def get_fusion_fidelity(self, queue_depth: int) -> str:
        """Fusion fidelity knob.

        Returns:
            'full': Run complete fusion pipeline.
            'skip': Reuse last fused frame (not yet implemented).
        """
        if self.policy in ('static_heavy', 'static_light'):
            self.last_fusion_fidelity = 'full'
            return 'full'

        if queue_depth > self.queue_depth_thresh:
            self.last_fusion_fidelity = 'skip'
            return 'skip'

        self.last_fusion_fidelity = 'full'
        return 'full'
