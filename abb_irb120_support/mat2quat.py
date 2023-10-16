import rospy
import tf.transformations as tf_transform
import numpy as np

# Define the rotation matrix
R_3x3 = np.array([
    [0.0047183,   -0.999864,  -0.0158078], 
    [-0.999987, -0.00474907,   0.0019095],
    [-0.00198431,   0.0157986,   -0.999873]
])

# Convert the 3x3 rotation matrix to a 4x4 transformation matrix
R = np.eye(4)
R[:3, :3] = R_3x3

# Convert rotation matrix to quaternion
quaternion = tf_transform.quaternion_from_matrix(R)

print("Quaternion:")
print("x:", quaternion[0])
print("y:", quaternion[1])
print("z:", quaternion[2])
print("w:", quaternion[3])