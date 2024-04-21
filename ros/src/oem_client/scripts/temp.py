import numpy as np
from scipy.spatial.transform import Rotation as R

def transform_odometry(odom_pos, odom_orient_deg, translation, rotation_deg):
    """
    Transforms odometry coordinates from one frame to another.

    Args:
        odom_pos (array-like): The position of the odometry in the original frame, [x, y, z].
        odom_orient_deg (array-like): The orientation of the odometry in the original frame in degrees, [roll, pitch, yaw].
        translation (array-like): The translation to apply, [dx, dy, dz].
        rotation_deg (array-like): The rotation to apply in degrees, [roll, pitch, yaw].

    Returns:
        np.array: The transformed position in the new frame.
    """
    # Convert orientation from degrees to radians
    odom_orient_rad = np.radians(odom_orient_deg)

    # Apply translation
    translated_pos = np.array(odom_pos) + np.array(translation)

    # Convert rotation from degrees to radians and create a rotation object
    rotation_rad = np.radians(rotation_deg)
    rot = R.from_euler('xyz', rotation_rad)

    # Apply rotation to the translated position
    transformed_pos = rot.apply(translated_pos)

    return transformed_pos

# Example usage
odom_pos = [-2.0, 0.0, -1.0]  # Original odometry position
odom_orient_deg = [0, 0, -90]  # Original odometry orientation (roll, pitch, yaw in degrees)
translation = [-2.0, 0.0, -1.5]  # Translation to apply
rotation_deg = [0, 0, -90]  # Rotation to apply (roll, pitch, yaw in degrees)

# Transform the odometry
transformed_pos = transform_odometry(odom_pos, odom_orient_deg, translation, rotation_deg)
print("Transformed position:", transformed_pos)