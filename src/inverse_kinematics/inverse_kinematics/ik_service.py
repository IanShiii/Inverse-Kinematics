import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from urdf_parser_py.urdf import URDF
from interfaces.srv import InverseKinematics
from ament_index_python.packages import get_package_share_directory
from inverse_kinematics.ik_solver import *
import os
import numpy as np
from typing import List

class IkServiceNode(Node):
    def __init__(self):
        super().__init__("ik_service_node")
        self.srv = self.create_service(InverseKinematics, "solve_ik", self.handle_solve_ik)
        self.joint_1_pose_pub = self.create_publisher(PoseStamped, "joint_1_pose", 10)
        self.joint_2_pose_pub = self.create_publisher(PoseStamped, "joint_2_pose", 10)
        self.joint_3_pose_pub = self.create_publisher(PoseStamped, "joint_3_pose", 10)
        self.joint_4_pose_pub = self.create_publisher(PoseStamped, "joint_4_pose", 10)
        self.joint_5_pose_pub = self.create_publisher(PoseStamped, "joint_5_pose", 10)
        self.joint_6_pose_pub = self.create_publisher(PoseStamped, "joint_6_pose", 10)
        self.joint_pose_publishers = [
            self.joint_1_pose_pub,
            self.joint_2_pose_pub,
            self.joint_3_pose_pub,
            self.joint_4_pose_pub,
            self.joint_5_pose_pub,
            self.joint_6_pose_pub,
        ]
        try:
            urdf_file = os.path.join(get_package_share_directory("arm_description"), "urdf", "arm.urdf")
            self.get_logger().info(f"Using URDF from arm_description: {urdf_file}")
        except Exception as e:
            self.get_logger().error(f"Could not locate URDF file: {e}")

        self.urdf : URDF = URDF.from_xml_file(urdf_file)

        joint_transforms: List[np.ndarray] = []
        end_efector_transform: np.ndarray

        for joint in self.urdf.joints:
            origin = getattr(joint, "origin")
            xyz = getattr(origin, "xyz")
            rpy = getattr(origin, "rpy")
            # log the joint transformations
            self.get_logger().info(f"Joint {joint.name} origin xyz: {xyz}, rpy: {rpy}")
            if not joint.type == "fixed":
                joint_transforms.append(pose_to_transformation_matrix(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]))
            else:
                end_efector_transform = pose_to_transformation_matrix(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2])
        
        joint_axis: List[np.ndarray] = []

        for joint in self.urdf.joints:
            if not joint.type == "fixed":
                axis = getattr(joint, "axis")
                self.get_logger().info(f"Joint {joint.name} axis: {axis}")
                joint_axis.append(np.array(axis) / np.linalg.norm(axis))

        self.ik_solver = IKSolver(joint_transforms, joint_axis, end_efector_transform)
        self.get_logger().info("IK service ready")

    def handle_solve_ik(self, request, response):
        self.get_logger().debug(f"Received IK request: {request}")
        target_pose_geometry_msg = request.target_pose
        target_pose_np = np.array([
            target_pose_geometry_msg.pose.position.x,
            target_pose_geometry_msg.pose.position.y,
            target_pose_geometry_msg.pose.position.z,
            *rotation_matrix_to_rpy(rpy_to_rotation_matrix(
                target_pose_geometry_msg.pose.orientation.x,
                target_pose_geometry_msg.pose.orientation.y,
                target_pose_geometry_msg.pose.orientation.z,
            ))
        ])

        # test_angles = [1.0, 0.2, 0.3, 0.3, 0.4, 0.4]

        try:
            joint_angles, success = self.ik_solver.solve_ik(target_pose_np)
            response.joint_angles = joint_angles
            # response.joint_angles = test_angles
            response.success = True
            # response.success = success

            for i in range(6):
                joint_pose = self.ik_solver.get_joint_pose(i, joint_angles)
                joint_pose_stamped = PoseStamped()
                joint_pose_stamped.header.frame_id = "base_link"
                joint_pose_stamped.pose.position.x = joint_pose[0]
                joint_pose_stamped.pose.position.y = joint_pose[1]
                joint_pose_stamped.pose.position.z = joint_pose[2]
                # convert rpy to quaternion
                quat = rpy_to_quaternion(joint_pose[3], joint_pose[4], joint_pose[5])
                joint_pose_stamped.pose.orientation.x = quat[0]
                joint_pose_stamped.pose.orientation.y = quat[1]
                joint_pose_stamped.pose.orientation.z = quat[2]
                joint_pose_stamped.pose.orientation.w = quat[3]
                self.joint_pose_publishers[i].publish(joint_pose_stamped)

        except Exception as e:
            response.joint_angles = []
            response.success = False
            response.message = f"IK exception: {e}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IkServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
