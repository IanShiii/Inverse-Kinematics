from typing import List
import numpy as np

def get_rotation_around_x_matrix(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0],
                    [0, c,-s],
                    [0, s, c]])

def get_rotation_around_y_matrix(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[ c, 0, s],
                    [ 0, 1, 0],
                    [-s, 0, c]])

def get_rotation_around_z_matrix(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c,-s, 0],
                    [s, c, 0],
                    [0, 0, 1]])

def rpy_to_rotation_matrix(roll, pitch, yaw):
    return get_rotation_around_z_matrix(yaw) @ get_rotation_around_y_matrix(pitch) @ get_rotation_around_x_matrix(roll)

def rotation_matrix_to_rpy(rotation_matrix):
    sy = np.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])
    singular = sy < 1e-6

    # Handle singularity in the case of gimbal lock
    if not singular:
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
        yaw = 0

    return roll, pitch, yaw

def skew(vector: np.ndarray):
    return np.array([[0, -vector[2], vector[1]],
                     [vector[2], 0, -vector[0]],
                     [-vector[1], vector[0], 0]])

# Using Rodrigues' rotation formula
def get_rotation_matrix_around_axis(axis: np.ndarray, angle: float):
    axis = axis / np.linalg.norm(axis)
    return (np.eye(3) * np.cos(angle)) + ((1 - np.cos(angle)) * np.outer(axis, axis)) + (np.sin(angle) * skew(axis))

def pose_to_transformation_matrix(x, y, z, roll, pitch, yaw):
    rotation_matrix = rpy_to_rotation_matrix(roll, pitch, yaw)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = np.array([x, y, z])
    return transformation_matrix

class IKSolver:
    def __init__(self, joint_transformations: List[np.ndarray], joint_axes: List[np.ndarray], end_effector_transformation: np.ndarray):
        # Relative transformations from one joint to the next
        self.joint_transformations = joint_transformations
        self.joint_axes = joint_axes
        self.end_effector_transformation = end_effector_transformation

    def get_transformation_matrix_for_joint(self, joint_index: int, joint_angle: float):
        rotation_from_joint_angle_matrix = np.eye(4)
        rotation_from_joint_angle_matrix[:3, :3] = get_rotation_matrix_around_axis(self.joint_axes[joint_index], joint_angle)
        return rotation_from_joint_angle_matrix @ self.joint_transformations[joint_index]
    
    def get_joint_position(self, joint_index: int, joint_angles: List[float]):
        joint_transformation_matrix = np.eye(4)
        for i in range(joint_index + 1):
            joint_transformation_matrix = self.get_transformation_matrix_for_joint(i, joint_angles[i]) @ joint_transformation_matrix
        
        return joint_transformation_matrix[:3, 3], rotation_matrix_to_rpy(joint_transformation_matrix[:3, :3])
    
    def get_end_effector_pose(self, joint_angles: List[float]):
        end_effector_transformation_matrix = np.eye(4)
        for i in range(len(self.joint_transformations)):
            end_effector_transformation_matrix = self.get_transformation_matrix_for_joint(i, joint_angles[i]) @ end_effector_transformation_matrix
        
        end_effector_transformation_matrix = self.end_effector_transformation @ end_effector_transformation_matrix

        return end_effector_transformation_matrix[:3, 3], rotation_matrix_to_rpy(end_effector_transformation_matrix[:3, :3])
        


    
