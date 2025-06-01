# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from opencda.core.plan.planer_debug_helper \
    import PlanDebugHelper


class EdgeDebugHelper(PlanDebugHelper):
    """This class aims to save statistics for platoon behaviour

    Parameters
    ----------
    actor_id : int
        The actor ID of the selected vehcile.

    Attributes
    ----------
    time_gap_list : list
        The list containing intra-time-gap(s) of all time-steps.

    dist_gap_list : list
        The list containing distance gap(s) of all time-steps.
    """

    def __init__(self, actor_id):
        super(EdgeDebugHelper, self).__init__(actor_id)

        self.algorithm_time_list = [[]]
        self.tracking_time_list = [[]]
        self.prediction_time_list = [[]]
        

    def update_edge(self, algorithm_time_step=None, tracking_time=None, prediction_time=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        """
        self.algorithm_time_list[0].append(algorithm_time_step)
        self.tracking_time_list[0].append(tracking_time)
        self.prediction_time_list[0].append(prediction_time)

