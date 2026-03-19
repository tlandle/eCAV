# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Localization Metrics

Tracks localization quality metrics: position error, yaw error, ATE, RPE.
Used by vehicle managers to evaluate localization performance.
"""
import numpy as np
import matplotlib
import warnings

import matplotlib.pyplot as plt


class LocalizationMetrics(object):
    """
    Tracks localization accuracy comparing GNSS, Kalman filter, and ground truth.

    Users can apply this class to track x, y coordinate trajectory, yaw angle
    and vehicle speed from GNSS raw measurements, Kalman filter, and ground truth.
    Error plotting and standard metrics (ATE, RPE) are computed.

    Attributes
        show_animation : boolean
            Indicator of whether to visulize animation.
        x_scale : float
            The scale of x coordinates.
        y_scale : float
            The scale of y coordinates.
        gnss_x : list
            The list of recorded gnss x coordinates.
        gnss_y : list
            The list of recorded gnss y coordinates.
        gnss_yaw : list
            The list of recorded gnss yaw angles.
        gnss_speed : list
            The list of recorded gnss speed values.
        filter_x : list
            The list of filtered x coordinates.
        filter_y : list
            The list of filtered y coordinates.
        filter_yaw : list
            The list of filtered yaw angles.
        filter_speed : list
            The list of filtered speed values.
        gt_x : list
            The list of ground truth x coordinates.
        gt_y : list
            The list of ground truth y coordinates.
        gt_yaw : list
            The list of ground truth yaw angles.
        gt_speed : list
            The list of ground truth speed values.
        hxEst : list
            The filtered x y coordinates.
        hTrue : list
            The true x y coordinates.
        hz : list
            The gnss detected x y coordinates.
        actor_id : int
            The list of ground truth speed values.
    """

    def __init__(self, config_yaml, actor_id):

        self.show_animation = config_yaml['show_animation']
        self.x_scale = config_yaml['x_scale']
        self.y_scale = config_yaml['y_scale']

        # off-line plotting
        self.gnss_x = []
        self.gnss_y = []
        self.gnss_yaw = []
        self.gnss_spd = []

        self.filter_x = []
        self.filter_y = []
        self.filter_yaw = []
        self.filter_spd = []

        self.gt_x = []
        self.gt_y = []
        self.gt_yaw = []
        self.gt_spd = []

        # online animation
        # filtered x y coordinates
        self.hxEst = np.zeros((2, 1))
        # gt x y coordinates
        self.hTrue = np.zeros((2, 1))
        # gnss x y coordinates
        self.hz = np.zeros((2, 1))

        self.actor_id = actor_id

    def run_step(self, gnss_x, gnss_y, gnss_yaw, gnss_spd,
                 filter_x, filter_y, filter_yaw, filter_spd,
                 gt_x, gt_y, gt_yaw, gt_spd):
        """
        Run a single step to save and animate(optional) the localization data.

        Args:
            -gnss_x (float): GNSS detected x coordinate.
            -gnss_y (float): GNSS detected y coordinate.
            -gnss_yaw (float): GNSS detected yaw angle.
            -gnss_spd (float): GNSS detected speed value.
            -filter_x (float): Filtered x coordinates.
            -filter_y (float): Filtered y coordinates.
            -filter_yaw (float): Filtered yaw angle.
            -filter_spd (float): Filtered speed value.
            -gt_x (float): The ground truth x coordinate.
            -gt_y (float): The ground truth y coordinate.
            -gt_yaw (float): The ground truth yaw angle.
            -gt_spd (float): The ground truth speed value.

        """
        self.gnss_x.append(gnss_x)
        self.gnss_y.append(gnss_y)
        self.gnss_yaw.append(gnss_yaw)
        self.gnss_spd.append(gnss_spd / 3.6)

        self.filter_x.append(filter_x)
        self.filter_y.append(filter_y)
        self.filter_yaw.append(filter_yaw)
        self.filter_spd.append(filter_spd / 3.6)

        self.gt_x.append(gt_x)
        self.gt_y.append(gt_y)
        self.gt_yaw.append(gt_yaw)
        self.gt_spd.append(gt_spd / 3.6)

        if self.show_animation:
            # call backend setting here to solve the conflict between cv2 pyqt5
            # and pyplot qtagg
            try:
                matplotlib.use('TkAgg')
            except ImportError:
                pass
            xEst = np.array([filter_x, filter_y]).reshape(2, 1)
            zTrue = np.array([gt_x, gt_y]).reshape(2, 1)
            z = np.array([gnss_x, gnss_y]).reshape(2, 1)

            self.hxEst = np.hstack((self.hxEst, xEst))
            self.hz = np.hstack((self.hz, z))
            self.hTrue = np.hstack((self.hTrue, zTrue))

            plt.cla()
            plt.title('actor id %d localization trajectory' % self.actor_id)
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event', lambda event: [
                    plt.close() if event.key == 'escape' else None])

            plt.plot(self.hTrue[0, 1:].flatten() * self.x_scale,
                     self.hTrue[1, 1:].flatten() * self.y_scale, "-b",
                     label='groundtruth')
            plt.plot(self.hz[0, 1:] *
                     self.x_scale, self.hz[1, 1:] *
                     self.y_scale, ".g", label='gnss noise data')
            plt.plot(self.hxEst[0, 1:].flatten() * self.x_scale,
                     self.hxEst[1, 1:].flatten() * self.y_scale, "-r",
                     label='kf result')

            plt.axis("equal")
            plt.grid(True)
            plt.legend()
            plt.pause(0.001)

    def get_metrics(self):
        """
        Get aggregated localization metrics.

        Returns
        -------
        dict
            Dictionary with all localization metrics
        """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)

            # GNSS errors
            gnss_x_error = float(np.nanmean(np.abs(np.array(self.gt_x) - np.array(self.gnss_x))))
            gnss_y_error = float(np.nanmean(np.abs(np.array(self.gt_y) - np.array(self.gnss_y))))
            gnss_yaw_error = float(np.nanmean(np.abs(np.array(self.gt_yaw) - np.array(self.gnss_yaw))))

            # Filter errors
            filter_x_error = float(np.nanmean(np.abs(np.array(self.gt_x) - np.array(self.filter_x))))
            filter_y_error = float(np.nanmean(np.abs(np.array(self.gt_y) - np.array(self.filter_y))))
            filter_yaw_error = float(np.nanmean(np.abs(np.array(self.gt_yaw) - np.array(self.filter_yaw))))

            # Compute ATE (Absolute Trajectory Error)
            gt_positions = np.column_stack([self.gt_x, self.gt_y])
            filter_positions = np.column_stack([self.filter_x, self.filter_y])
            gnss_positions = np.column_stack([self.gnss_x, self.gnss_y])

            ate_filter = np.nan
            ate_gnss = np.nan
            if len(gt_positions) > 0 and len(filter_positions) > 0:
                position_errors_filter = np.linalg.norm(gt_positions - filter_positions, axis=1)
                ate_filter = float(np.sqrt(np.nanmean(position_errors_filter**2)))

                position_errors_gnss = np.linalg.norm(gt_positions - gnss_positions, axis=1)
                ate_gnss = float(np.sqrt(np.nanmean(position_errors_gnss**2)))

            # Compute RPE (Relative Pose Error)
            rpe_filter = self._compute_rpe(gt_positions, filter_positions)
            rpe_gnss = self._compute_rpe(gt_positions, gnss_positions)

            return {
                "actor_id": self.actor_id,
                "gnss_x_error_mean_m": gnss_x_error,
                "gnss_y_error_mean_m": gnss_y_error,
                "gnss_yaw_error_mean_deg": gnss_yaw_error,
                "filter_x_error_mean_m": filter_x_error,
                "filter_y_error_mean_m": filter_y_error,
                "filter_yaw_error_mean_deg": filter_yaw_error,
                "ate_gnss_m": ate_gnss if not np.isnan(ate_gnss) else None,
                "ate_filter_m": ate_filter if not np.isnan(ate_filter) else None,
                "rpe_gnss_m": rpe_gnss if not np.isnan(rpe_gnss) else None,
                "rpe_filter_m": rpe_filter if not np.isnan(rpe_filter) else None,
            }

    def evaluate(self):
        """
        Plot the localization related data points.

        Returns:
            -figures(matplotlib.pyplot.plot): The plot of
            localization related figures.
            -perform_txt(txt file): The localization related
            datas saved as text file.

        """
        figure, axis = plt.subplots(2, 2)
        figure.set_size_inches(10, 6)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            # x, y coordinates
            axis[0, 0].plot(self.gnss_x, self.gnss_y, ".g", label='gnss')
            axis[0, 0].plot(self.gt_x, self.gt_y, ".b", label='gt')
            axis[0, 0].plot(self.filter_x, self.filter_y, ".r", label='filter')
            axis[0, 0].legend(fontsize=10)
            axis[0, 0].set_title("x-y coordinates plotting")
            axis[0, 0].set_xlabel('x', fontdict={'weight': 'bold', 'fontsize': 13})
            axis[0, 0].set_ylabel('y', fontdict={'weight': 'bold', 'fontsize': 13})


            # yaw angle
            axis[0, 1].plot(np.arange(len(self.gnss_yaw)),
                            self.gnss_yaw, ".g", label='gnss')
            axis[0, 1].plot(np.arange(len(self.gt_yaw)),
                            self.gt_yaw, ".b", label='gt')
            axis[0, 1].plot(np.arange(len(self.filter_yaw)),
                            self.filter_yaw, ".r", label='filter')
            axis[0, 1].legend(fontsize=10)
            axis[0, 1].set_title("yaw angle(degree) plotting")
            axis[0, 1].set_xlabel('Time', fontdict={'weight': 'bold', 'fontsize': 13})
            axis[0, 1].set_ylabel('Angle', fontdict={'weight': 'bold', 'fontsize': 13})

            # speed
            axis[1, 0].plot(np.arange(len(self.gnss_spd)),
                            self.gnss_spd, ".g", label='gnss')
            axis[1, 0].plot(np.arange(len(self.gt_spd)),
                            self.gt_spd, ".b", label='gt')
            axis[1, 0].plot(np.arange(len(self.filter_spd)),
                            self.filter_spd, ".r", label='filter')
            axis[1, 0].legend(fontsize=10)
            axis[1, 0].set_title("speed(m/s) plotting")
            axis[1, 0].set_xlabel('Time', fontdict={'weight': 'bold', 'fontsize': 13})
            axis[1, 0].set_ylabel('Speed (m/s)', fontdict={'weight': 'bold', 'fontsize': 13})

            # error curve on x
            axis[1, 1].plot(np.arange(len(self.gnss_x)), np.array(
                self.gt_x) - np.array(self.gnss_x), "-g", label='gnss')
            axis[1, 1].plot(np.arange(len(self.filter_x)), np.array(
                self.gt_x) - np.array(self.filter_x), "-r", label='filter')
            axis[1, 1].legend(fontsize=10)
            axis[1, 1].set_title("error curve on x coordinates")
            axis[1, 1].set_xlabel('Time', fontdict={'weight': 'bold', 'fontsize': 13})
            axis[1, 1].set_ylabel('Error', fontdict={'weight': 'bold', 'fontsize': 13})


            plt.tight_layout()

            # Get metrics
            metrics = self.get_metrics()

            perform_txt = 'mean error for GNSS raw data on x-axis: %f (meter), ' \
                        'mean error for GNSS raw data on y-axis: %f (meter),' \
                        'mean error for GNSS raw data on yaw : %f (degree) \n'\
                        % (metrics['gnss_x_error_mean_m'],
                            metrics['gnss_y_error_mean_m'],
                            metrics['gnss_yaw_error_mean_deg'])

            perform_txt += 'mean error after data fusion on x-axis: %f (meter), ' \
                        'mean error after data fusion  on y-axis: %f (meter),' \
                        'mean error after data fusion yaw : %f (degree) \n' \
                        % (metrics['filter_x_error_mean_m'],
                            metrics['filter_y_error_mean_m'],
                            metrics['filter_yaw_error_mean_deg'])

            ate_gnss = metrics['ate_gnss_m'] if metrics['ate_gnss_m'] is not None else np.nan
            ate_filter = metrics['ate_filter_m'] if metrics['ate_filter_m'] is not None else np.nan
            rpe_gnss = metrics['rpe_gnss_m'] if metrics['rpe_gnss_m'] is not None else np.nan
            rpe_filter = metrics['rpe_filter_m'] if metrics['rpe_filter_m'] is not None else np.nan

            perform_txt += 'ATE (GNSS): %.4f (m), ATE (filtered): %.4f (m) \n' % (ate_gnss, ate_filter)
            perform_txt += 'RPE (GNSS): %.4f (m), RPE (filtered): %.4f (m) \n' % (rpe_gnss, rpe_filter)

            return figure, perform_txt, metrics

    def _compute_rpe(self, gt_positions, est_positions, delta=1):
        """
        Compute Relative Pose Error (RPE).

        RPE measures the local accuracy by comparing relative transformations
        between consecutive poses.

        Args:
            gt_positions: Ground truth (x, y) positions as Nx2 array
            est_positions: Estimated (x, y) positions as Nx2 array
            delta: Frame interval for computing relative poses

        Returns:
            float: RMSE of relative pose errors
        """
        if len(gt_positions) < delta + 1 or len(est_positions) < delta + 1:
            return np.nan

        rpe_errors = []
        for i in range(len(gt_positions) - delta):
            # Ground truth relative motion
            gt_rel = gt_positions[i + delta] - gt_positions[i]

            # Estimated relative motion
            est_rel = est_positions[i + delta] - est_positions[i]

            # Error in relative motion
            rel_error = np.linalg.norm(gt_rel - est_rel)
            rpe_errors.append(rel_error)

        if rpe_errors:
            return float(np.sqrt(np.mean(np.array(rpe_errors)**2)))
        return np.nan

    def serialize_debug_info(self, proto_debug_helper):
        # TODO: extend instead of append? or [:] = ?

        for obj in self.gnss_x:
            proto_debug_helper.gnss_x.append(obj)

        for obj in self.gnss_y:
            proto_debug_helper.gnss_y.append(obj)

        for obj in self.gnss_yaw:
            proto_debug_helper.gnss_yaw.append(obj)

        for obj in self.gnss_spd:
            proto_debug_helper.gnss_spd.append(obj)

        for obj in self.filter_x:
            proto_debug_helper.filter_x.append(obj)

        for obj in self.filter_y:
            proto_debug_helper.filter_y.append(obj)

        for obj in self.filter_yaw:
            proto_debug_helper.filter_yaw.append(obj)

        for obj in self.filter_spd:
            proto_debug_helper.filter_spd.append(obj)

        for obj in self.gt_x:
            proto_debug_helper.gt_x.append(obj)

        for obj in self.gt_y:
            proto_debug_helper.gt_y.append(obj)

        for obj in self.gt_yaw:
            proto_debug_helper.gt_yaw.append(obj)

        for obj in self.gt_spd:
            proto_debug_helper.gt_spd.append(obj)

    def deserialize_debug_info(self, proto_debug_helper):
        # call from Sim API to populate locally

        self.gnss_x.clear()
        for obj in proto_debug_helper.gnss_x:
            self.gnss_x.append(obj)

        self.gnss_y.clear()
        for obj in proto_debug_helper.gnss_y:
            self.gnss_y.append(obj)

        self.gnss_yaw.clear()
        for obj in proto_debug_helper.gnss_yaw:
            self.gnss_yaw.append(obj)

        self.gnss_spd.clear()
        for obj in proto_debug_helper.gnss_spd:
            self.gnss_spd.append(obj)

        self.filter_x.clear()
        for obj in proto_debug_helper.filter_x:
            self.filter_x.append(obj)

        self.filter_y.clear()
        for obj in proto_debug_helper.filter_y:
            self.filter_y.append(obj)

        self.filter_yaw.clear()
        for obj in proto_debug_helper.filter_yaw:
            self.filter_yaw.append(obj)

        self.filter_spd.clear()
        for obj in proto_debug_helper.filter_spd:
            self.filter_spd.append(obj)

        self.gt_x.clear()
        for obj in proto_debug_helper.gt_x:
            self.gt_x.append(obj)

        self.gt_y.clear()
        for obj in proto_debug_helper.gt_y:
            self.gt_y.append(obj)

        self.gt_yaw.clear()
        for obj in proto_debug_helper.gt_yaw:
            self.gt_yaw.append(obj)

        self.gt_spd.clear()
        for obj in proto_debug_helper.gt_spd:
            self.gt_spd.append(obj)


# Backward compatibility alias
LocDebugHelper = LocalizationMetrics
