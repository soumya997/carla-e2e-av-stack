#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

# --- Fix #1: Use keyword argument for queue_size to avoid TCP/IP warning ---
odom_in_map_pub = rospy.Publisher("/aloam_odom_in_map", Odometry, queue_size=10)
odom_trf_in_map_pub = rospy.Publisher("/aloam_odom_trf_in_map", Odometry, queue_size=10)

# The transform you provided is described as:
#   Quaternion (w, x, y, z): [9.99999902e-01, 9.84793956e-05, 6.88315138e-05, -4.26514447e-04]
#   Translation (x, y, z):   [1.24705909e+02, -1.98756426e+02, 1.09774383e-03]
#
# FROM V2
# Quaternion (x, y, z, w): [ 0.05495642 -0.00337278  0.99847951 -0.00266143]
# Translation (x, y, z): [ 207.40259583 -128.51522024  -25.47652147]
# FROM V3
# Translation vector (map to camera_init): [-4.85388541 72.69005811 25.47354793]
# Relative rotation quaternion (x, y, z, w): [ 0.0032524   0.05453023  0.00230243 -0.99850417]



# However, Rotation.from_quat() in scipy expects [x, y, z, w] order.
# So we reorder them carefully below:
# q_trf_ros = [9.99999902e-01, 9.84793956e-05, 6.88315138e-05, -4.26514447e-04]  # (w, x, y, z)
# q_trf_ros = [-0.00266143, 0.05495642, -0.00337278, 0.99847951]  # (w, x, y, z)
# q_trf_ros = [-0.99850417, 0.0032524, 0.05453023, 0.00230243]  # (w, x, y, z)
q_trf_ros = [5.36980261e-05, -4.21505555e-04, 1.37180098e-05, -9.99999720e-01 ]  # (w, x, y, z)


q_trf_scipy = [q_trf_ros[1], q_trf_ros[2], q_trf_ros[3], q_trf_ros[0]]         # -> (x, y, z, w)

# t_trf = [1.24705909e+02, -1.98756426e+02, 1.09774383e-03]
# t_trf = [207.40259583, -128.51522024, -25.47652147]
# t_trf = [-5.75512421, 73.73788432, 1.93015205]
t_trf = [ 2.02551783e+02, -5.58189340e+01, -4.85693137e-03]




def apply_transform(p_odom, q_odom, t, q):
    """
    Apply the transform:
      p_out = R(q) * p_odom + t
      q_out = q * q_odom   (if we consider q first in the chain, be mindful of order)
    where q, q_odom are in SciPy's (x, y, z, w) order.
    """

    # Step 1: Combine orientations
    #   R_total = R(q) * R(q_odom)
    #   NOTE: In scipy, multiplication is done right-to-left if you read R(q1)*R(q2)
    #         as applying q2 first, then q1. Adjust if your transform semantics differ.
    r_q = R.from_quat(q)
    r_odom = R.from_quat(q_odom)
    r_new = r_q * r_odom
    q_new = r_new.as_quat()  # still in (x, y, z, w)

    # Step 2: Rotate and translate position
    p_new = r_q.apply(p_odom) + t

    return p_new, q_new


def odom_callback(data):
    """
    Callback to handle new A-LOAM Odometry messages, transform them,
    and publish them in the map frame.
    """
    # 1) Publish a copy of the original A-LOAM odom, but with frame_id = "map"
    odom = Odometry()
    odom.header = data.header
    odom.header.frame_id = "map"
    odom.pose = data.pose  # (Optional) Copy the original pose if desired
    odom.twist = data.twist  # (Optional) Copy twist if desired
    odom_in_map_pub.publish(odom)

    # 2) Create another Odometry for the "transformed" version
    odom_trf = Odometry()
    odom_trf.header = data.header
    odom_trf.header.frame_id = "map"

    # Extract position & orientation from the incoming message
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    # Convert from ROS orientation (x, y, z, w) to a list
    # But be careful: orientation.x, .y, .z, .w -> that's already (x, y, z, w).
    # So if we feed that into SciPy, we are *already* in the correct order.
    q_odom_scipy = [orientation.x, orientation.y, orientation.z, orientation.w]

    p_odom = [position.x, position.y, position.z]

    # Apply the transform
    p_new, q_new = apply_transform(p_odom, q_odom_scipy, t_trf, q_trf_scipy)

    # Fill the new Odometry message
    odom_trf.pose.pose.position.x = p_new[0]
    odom_trf.pose.pose.position.y = p_new[1]
    odom_trf.pose.pose.position.z = p_new[2]

    # q_new is still in (x, y, z, w), so we can place it directly
    odom_trf.pose.pose.orientation.x = q_new[0]
    odom_trf.pose.pose.orientation.y = q_new[1]
    odom_trf.pose.pose.orientation.z = q_new[2]
    odom_trf.pose.pose.orientation.w = q_new[3]

    # (Optional) copy over twist if it makes sense in the transformed frame,
    # or set it to zero if you only care about pose, etc.
    # odom_trf.twist = data.twist  

    odom_trf_in_map_pub.publish(odom_trf)


def main():
    rospy.init_node("odom_transform")
    # --- Fix #1: Use keyword argument for queue_size to avoid TCP/IP warning ---
    rospy.Subscriber("/aft_mapped_to_init_high_frec", Odometry, odom_callback, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
