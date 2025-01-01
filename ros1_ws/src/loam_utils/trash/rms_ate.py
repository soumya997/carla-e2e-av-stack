import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Global lists to store positions
carla_positions = []
transformed_aloam_positions = []

# # Transformation (calculated earlier)
# source_quat = [-9.835933849850075e-05, -3.1290452066124376e-05, 0.00032596024859776415, 0.9999999415481308]
# source_pos = [0.01693921645885583, -0.0019361239704956653, 0.0006106553212857983]
# target_quat = [1.2915154991739152e-07, 3.7550912103400815e-05, -0.00010055051711756224, 0.9999999942397528]
# target_pos = [124.72284698486328, -198.7583770751953, 0.0017056845827028155]

# source_rotation = R.from_quat(source_quat).as_matrix()
# target_rotation = R.from_quat(target_quat).as_matrix()

# relative_rotation = target_rotation @ np.linalg.inv(source_rotation)
# relative_translation = np.array(target_pos) - relative_rotation @ np.array(source_pos)


# def transform_pose(position):
#     """Applies the relative transformation to an A-LOAM position."""
#     return relative_rotation @ np.array(position) + relative_translation


def calculate_rms_ate_and_scale_error():
    """Calculates RMS ATE and Scale Error."""
    if len(carla_positions) != len(transformed_aloam_positions):
        rospy.logerr("Mismatch in trajectory lengths. Cannot compute metrics.")
        return

    # RMS ATE
    errors = [
        np.linalg.norm(np.array(carla) - np.array(trans_aloam))
        for carla, trans_aloam in zip(carla_positions, transformed_aloam_positions)
    ]
    rms_ate = np.sqrt(np.mean(np.square(errors)))

    # Trajectory lengths
    def compute_trajectory_length(positions):
        return sum(
            np.linalg.norm(np.array(positions[i]) - np.array(positions[i - 1]))
            for i in range(1, len(positions))
        )

    carla_length = compute_trajectory_length(carla_positions)
    aloam_length = compute_trajectory_length(transformed_aloam_positions)
    scale_error = abs((aloam_length - carla_length) / carla_length) * 100

    rospy.loginfo(f"RMS ATE: {rms_ate:.4f} m")
    rospy.loginfo(f"Scale Error: {scale_error:.2f} %")


def callback(carla_odom, aloam_odom):
    print("inside")
    """Callback function for synchronized CARLA and A-LOAM odometry."""
    # CARLA position
    carla_position = [
        carla_odom.pose.pose.position.x,
        carla_odom.pose.pose.position.y,
        carla_odom.pose.pose.position.z,
    ]
    carla_positions.append(carla_position)

    # A-LOAM position
    aloam_position = [
        aloam_odom.pose.pose.position.x,
        aloam_odom.pose.pose.position.y,
        aloam_odom.pose.pose.position.z,
    ]

    # Transform A-LOAM position to CARLA frame
    # transformed_position = transform_pose(aloam_position)
    transformed_aloam_positions.append(aloam_position)

    # Periodically calculate metrics
    if len(carla_positions) >= 10:  # Adjust frequency as needed
        calculate_rms_ate_and_scale_error()


def main():
    rospy.init_node("trajectory_transform_and_metrics")

    # Subscribers for CARLA and A-LOAM odometry
    carla_sub = Subscriber("/carla/ego_vehicle/odometry", Odometry)
    aloam_sub = Subscriber("/odometry_from_dynamic_tf", Odometry)

    # Synchronize messages with approximate time synchronization
    ats = ApproximateTimeSynchronizer([carla_sub, aloam_sub], queue_size=10, slop=0.1)
    ats.registerCallback(callback)

    rospy.spin()


if __name__ == "__main__":
    main()
