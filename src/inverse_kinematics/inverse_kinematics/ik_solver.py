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

def rpy_to_quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return np.array([qx, qy, qz, qw])

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
    def __init__(self, default_joint_transformations: List[np.ndarray], joint_axes: List[np.ndarray], end_effector_transformation: np.ndarray):
        # Relative transformations from one joint to the next
        self.default_joint_transformations = default_joint_transformations
        self.joint_axes = joint_axes
        self.end_effector_transformation = end_effector_transformation

    def get_transformation_matrix_for_joint(self, joint_index: int, joint_angle: float):
        rotation_from_joint_angle_matrix = np.eye(4)
        rotation_from_joint_angle_matrix[:3, :3] = get_rotation_matrix_around_axis(self.joint_axes[joint_index], joint_angle)
        return self.default_joint_transformations[joint_index] @ rotation_from_joint_angle_matrix #
    
    def get_joint_pose(self, joint_index: int, joint_angles: List[float]):
        joint_transformation_matrix = np.eye(4)
        for i in range(joint_index + 1):
            joint_transformation_matrix = joint_transformation_matrix @ self.get_transformation_matrix_for_joint(i, joint_angles[i]) #
        
        xyz = joint_transformation_matrix[:3, 3]
        rpy = rotation_matrix_to_rpy(joint_transformation_matrix[:3, :3])

        return (*xyz, *rpy)
    
    def get_end_effector_pose(self, joint_angles: List[float]):
        end_effector_transformation_matrix = np.eye(4)
        for i in range(len(self.default_joint_transformations)):
            end_effector_transformation_matrix = end_effector_transformation_matrix @ self.get_transformation_matrix_for_joint(i, joint_angles[i]) #
        
        end_effector_transformation_matrix = end_effector_transformation_matrix @ self.end_effector_transformation

        xyz = end_effector_transformation_matrix[:3, 3]
        rpy = rotation_matrix_to_rpy(end_effector_transformation_matrix[:3, :3])

        return (*xyz, *rpy)
    
    def get_jacobian(self, joint_angles: List[float]):
        num_joints = len(self.default_joint_transformations)
        jacobian = np.zeros((6, num_joints))
        
        end_effector_pose = self.get_end_effector_pose(joint_angles)
        end_effector_translation = np.array(end_effector_pose[:3])
        
        for i in range(num_joints):
            joint_pose = self.get_joint_pose(i, joint_angles)

            axis_rotation = rpy_to_rotation_matrix(joint_pose[3], joint_pose[4], joint_pose[5])
            joint_axis = axis_rotation @ self.joint_axes[i]

            joint_translation = np.array(joint_pose[:3])
            
            # Linear part
            jacobian[:3, i] = np.cross(joint_axis, end_effector_translation - joint_translation)
            # Angular part
            jacobian[3:, i] = joint_axis
        
        return jacobian
    
    def solve_ik(self, target_pose: np.ndarray, initial_guess: List[float] = None, max_iterations: int = 5000, tolerance: float = 0.02):
        if initial_guess is None:
            initial_guess = [0.0] * len(self.default_joint_transformations)
        
        joint_angles = np.array(initial_guess, dtype=float)
        
        for iteration in range(max_iterations):
            current_end_effector_pose = self.get_end_effector_pose(joint_angles)
            current_end_effector_translation = np.array(current_end_effector_pose[:3])
            current_end_effector_rotation = rpy_to_rotation_matrix(*current_end_effector_pose[3:])
            
            target_end_effector_translation = target_pose[:3]
            target_end_effector_rotation = rpy_to_rotation_matrix(*target_pose[3:])
            
            translation_error = target_end_effector_translation - current_end_effector_translation
            rotation_error = current_end_effector_rotation.T @ target_end_effector_rotation

            axis = np.array([
                rotation_error[2, 1] - rotation_error[1, 2],
                rotation_error[0, 2] - rotation_error[2, 0],
                rotation_error[1, 0] - rotation_error[0, 1]
            ]) / 2.0

            angle = np.arccos((np.trace(rotation_error) - 1) / 2.0)
            rotation_error_vector = axis * angle
            
            # Compute the error vector
            # error_vector = np.concatenate((translation_error, rotation_matrix_to_rpy(rotation_error)))
            error_vector = np.concatenate((translation_error, rotation_error_vector))
            
            if np.linalg.norm(error_vector) < tolerance:
                return joint_angles.tolist(), True
            
            jacobian = self.get_jacobian(joint_angles)
            # Use pseudo-inverse to compute the change in joint angles with a damping term to push eigenvalues away from 0
            delta_angles = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + 0.001 * np.eye(6)) @ error_vector
            
            joint_angles += 0.005 * delta_angles
        
        return joint_angles.tolist(), False
        


    
