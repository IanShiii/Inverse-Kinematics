import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from urdf_parser_py.urdf import URDF
from interfaces.srv import InverseKinematics
from ament_index_python.packages import get_package_share_directory
import os


def solve_ik(pose: Pose):
    # Example: return a fixed 6-DOF zero position (modify for your robot)
    num_joints = 6
    joint_angles = [0.0] * num_joints

    # TODO: compute actual joint_angles from pose
    success = True
    message = "IK solved (placeholder) - replace solve_ik_impl with real solver."
    return joint_angles, success, message


class IkServiceNode(Node):
    def __init__(self):
        super().__init__("ik_service_node")
        self.srv = self.create_service(InverseKinematics, "solve_ik", self.handle_solve_ik)
        try:
            urdf_file = os.path.join(get_package_share_directory("arm_description"), "urdf", "arm.urdf.xacro")
            self.get_logger().info(f"Using URDF from arm_description: {urdf_file}")
        except Exception as e:
            self.get_logger().error(f"Could not locate URDF file: {e}")

        self.urdf : URDF = URDF.from_xml_file(urdf_file)
        for joint in robot.joints:
            print("\nJoint:", joint.name)
            print("  Type:", joint.joint_type)
            print("  Parent:", joint.parent)
            print("  Child:", joint.child)
        self.get_logger().info("IK service ready")

    def handle_solve_ik(self, request, response):
        self.get_logger().debug(f"Received IK request: {request}")
        try:
            joint_angles, success, message = solve_ik(request.pose)
            response.joint_angles = joint_angles
            response.success = bool(success)
            response.message = message
            self.get_logger().info(f"IK result: success={success}, message='{message}'")
        except Exception as e:
            response.joint_angles = []
            response.success = False
            response.message = f"IK exception: {e}"
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IkServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down IK service node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
