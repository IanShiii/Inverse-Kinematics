import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from urdf_parser_py.urdf import URDF
from interfaces.srv import InverseKinematics
from ament_index_python.packages import get_package_share_directory
import os


def solve_ik(pose: Pose):
    num_joints = 6
    joint_angles = [0.0] * num_joints

    # TODO: compute actual joint_angles from pose
    success = True
    return joint_angles, success 


class IkServiceNode(Node):
    def __init__(self):
        super().__init__("ik_service_node")
        self.srv = self.create_service(InverseKinematics, "solve_ik", self.handle_solve_ik)
        try:
            urdf_file = os.path.join(get_package_share_directory("arm_description"), "urdf", "arm.urdf")
            self.get_logger().info(f"Using URDF from arm_description: {urdf_file}")
        except Exception as e:
            self.get_logger().error(f"Could not locate URDF file: {e}")

        self.urdf : URDF = URDF.from_xml_file(urdf_file)
        self.get_logger().info(f"Loaded URDF with {len(self.urdf.links)} links and {len(self.urdf.joints)} joints.")
        # for joint in self.urdf.joints:
        #     self.get_logger().info(f"\nJoint: {joint.name}")
        self.get_logger().info("IK service ready")

    def handle_solve_ik(self, request, response):
        self.get_logger().debug(f"Received IK request: {request}")
        try:
            joint_angles, success = solve_ik(request.target_pose)
            response.joint_angles = joint_angles
            response.success = bool(success)
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
