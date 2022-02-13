import math
import numpy as np
from scipy.spatial.transform import Rotation as R

def dist2d(target_position, new_position):
    """Calculate the distance between two Points (positions)"""
    diff = target_position - new_position
    return math.hypot(diff[0], diff[1])

def dist3d(target_position, new_position):
    """Calculate the distance between two Points (positions)"""
    diff = target_position - new_position
    return math.hypot(diff[0], diff[1], diff[2])

def check_speed(speed):
    """Safety rule for speed"""
    if speed >= 0.02:
        return 0.02
    else:
        return speed

def pos2arr (pos):
    return np.array([pos.x, pos.y, pos.z])

def quat2arr (quat):
    return np.array([quat.x, quat.y, quat.z, quat.w])

def quatmul(quat0, quat1):
    x0, y0, z0, w0 = quat0
    x1, y1, z1, w1 = quat1
    return np.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                     -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=np.float64)

def get_rotation (vec1, vec2):
    """Get vector1 to vector2 rotation. numpy arrays given"""
    vec1 = vec1 / np.linalg.norm(vec1)
    vec2 = vec2 / np.linalg.norm(vec2)

    axis = np.cross(vec1, vec2) # get rotation axis
    theta = np.arcsin(np.linalg.norm(axis)) # get rotation angle
    axis = axis/np.linalg.norm(axis)
    rotvec = theta * axis
    rot = R.from_rotvec(rotvec)

    if np.linalg.norm(rot.apply(vec1) - vec2) > 0.01:
        theta = np.pi - theta
        rotvec = theta * axis
        rot = R.from_rotvec(rotvec)
    return rot
