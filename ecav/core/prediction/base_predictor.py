"""Abstract base class for trajectory predictors."""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

from abc import ABC, abstractmethod
from typing import Dict, List, Optional

from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from ecav.core.prediction.obstacle_prediction import ObstaclePrediction


class BasePredictor(ABC):
    """Interface for all trajectory predictors in the edge pipeline.

    Every predictor takes tracked obstacle trajectories (from a tracker)
    and produces future trajectory predictions consumed by the behavior
    planner's collision check.

    Implementations: LinearPredictorManager, SMARTPredictorManager3D.
    """

    @abstractmethod
    def generate_predicted_trajectories(
            self,
            tracked_obstacles_trajectories: Dict[int, ObstacleTrajectory],
            source_tick: Optional[int] = None,
            publish_tick: Optional[int] = None,
    ) -> List[ObstaclePrediction]:
        """Predict future trajectories for tracked obstacles.

        Args:
            tracked_obstacles_trajectories: track_id -> ObstacleTrajectory.
                trajectory[0] is newest, trajectory[-1] is oldest.
            source_tick: Simulation tick of the latest detection frame.
            publish_tick: Simulation tick when the edge pushes predictions.

        Returns:
            One ObstaclePrediction per predicted obstacle, each containing
            a list of future Transform objects in world coordinates.
        """
        ...
