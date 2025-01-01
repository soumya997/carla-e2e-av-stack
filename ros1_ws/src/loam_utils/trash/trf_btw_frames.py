import numpy as np
import tf.transformations as tfm
from scipy.spatial.transform import Rotation as R

# Map pose in odom
pos_map = np.array([202.5500030517578, -55.84000015258789, 0.001503581996075809])
quat_map = np.array([0.0, -0.0, -0.9999999999977304, 2.130528872050454e-06])

# Camera_init pose in odom
# pos_camera = np.array([207.40388846416772, -128.53005826370978, -25.472044343848086])
# quat_camera = np.array([0.05453022434810571, -0.0032525157026825563, 0.9985041636320053, -0.0023045571589246634])
pos_camera = np.array([0.0017880477933616135, 0.021065766860687063, 0.006359587205423178])
quat_camera = np.array([1.3718907785582185e-05, 0.0004215055259505539, -5.156749786888737e-05, 0.9999997199120154])


# Invert camera pose
quat_camera_inv = tfm.quaternion_conjugate(quat_camera)

# Rotate the position vector using the inverted quaternion
rot_camera_inv = R.from_quat(quat_camera_inv)
pos_camera_inv = -rot_camera_inv.apply(pos_camera)

# Combine map pose and inverted camera pose
quat_result = tfm.quaternion_multiply(quat_map, quat_camera_inv)

# Rotate the inverted camera position by the map orientation
rot_map = R.from_quat(quat_map)
pos_result = pos_map + rot_map.apply(pos_camera_inv)

print("Transformation from camera_init to map:")
print("Position:", pos_result)
print("Orientation:", quat_result)

