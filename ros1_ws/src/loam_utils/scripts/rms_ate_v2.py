#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import threading

class OdometryEvaluator:
    def __init__(self):
        # Subscribers
        self.sub_est = rospy.Subscriber("/odometry_from_dynamic_tf", Odometry,
                                        self.callback_est, queue_size=1)
        self.sub_gt  = rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry,
                                        self.callback_gt, queue_size=1)
        
        self.lock = threading.Lock()
        
        # Store data as dictionaries keyed by timestamp (float or ROS time)
        self.est_data = {}  # {stamp: (x, y, z)}
        self.gt_data  = {}  # {stamp: (x, y, z)}

    def callback_est(self, msg):
        """
        Callback for the A-LOAM odometry (estimated).
        """
        with self.lock:
            t = msg.header.stamp.to_sec()
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z
            self.est_data[t] = (px, py, pz)

    def callback_gt(self, msg):
        """
        Callback for the CARLA odometry (ground truth).
        """
        with self.lock:
            t = msg.header.stamp.to_sec()
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z
            self.gt_data[t] = (px, py, pz)

    def compute_metrics(self):
        """
        Match time stamps (nearest neighbor), compute RMS ATE & scale error.
        """
        with self.lock:
            # Convert to sorted lists for matching
            est_times = sorted(self.est_data.keys())
            gt_times  = sorted(self.gt_data.keys())
            
            # If empty, just return
            if len(est_times) == 0 or len(gt_times) == 0:
                rospy.logwarn("No data available yet.")
                return None, None
            
            matched_est_positions = []
            matched_gt_positions  = []
            
            i_gt = 0
            for t_est in est_times:
                # Find closest ground-truth timestamp
                while i_gt < (len(gt_times) - 1) and \
                        abs(gt_times[i_gt+1] - t_est) < abs(gt_times[i_gt] - t_est):
                    i_gt += 1

                # Now gt_times[i_gt] is the closest
                matched_est_positions.append(self.est_data[t_est])
                matched_gt_positions.append(self.gt_data[gt_times[i_gt]])
            
            # Convert to numpy arrays
            matched_est_positions = np.array(matched_est_positions)
            matched_gt_positions  = np.array(matched_gt_positions)

            # ========== (A) Compute RMS ATE ==========
            differences = matched_gt_positions - matched_est_positions
            sq_errors = np.sum(differences**2, axis=1)
            rms_ate = np.sqrt(np.mean(sq_errors))

            # ========== (B) Compute Scale Error ==========
            # 1) Path length for ground truth
            gt_path_length = self._compute_path_length(gt_times, self.gt_data)
            # 2) Path length for estimated
            est_path_length = self._compute_path_length(est_times, self.est_data)
            # 3) Scale factor s = est_path_length / gt_path_length
            #    Scale error is |s - 1|
            if gt_path_length > 0:
                scale_factor = est_path_length / gt_path_length
                scale_error  = abs(scale_factor - 1.0)
            else:
                scale_factor = 0.0
                scale_error = 0.0

            return rms_ate, scale_error

    def _compute_path_length(self, times, data_dict):
        """Compute total path length using successive positions in time order."""
        if len(times) < 2:
            return 0.0

        path_length = 0.0
        prev_pos = np.array(data_dict[times[0]])
        for i in range(1, len(times)):
            curr_pos = np.array(data_dict[times[i]])
            path_length += np.linalg.norm(curr_pos - prev_pos)
            prev_pos = curr_pos
        return path_length


def main():
    rospy.init_node("odometry_evaluation_node")

    evaluator = OdometryEvaluator()
    rate = rospy.Rate(1.0)  # once per second

    while not rospy.is_shutdown():
        rms_ate, scale_error = evaluator.compute_metrics()
        if rms_ate is not None:
            rospy.loginfo("RMS ATE = %.4f m", rms_ate)
            rospy.loginfo("Scale Error = %.4f", scale_error)
        rate.sleep()

if __name__ == "__main__":
    main()
