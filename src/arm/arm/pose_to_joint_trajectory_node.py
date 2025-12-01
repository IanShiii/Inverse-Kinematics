import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from interfaces.srv import InverseKinematics
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class PoseToJointTrajectoryNode(Node):
    def __init__(self):
        super().__init__('pose_to_joint_trajectory_node')

        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 1)

        # Wait for IK service
        self.get_logger().info("Waiting for IK service...")
        self.ik_service = self.create_client(InverseKinematics, "solve_ik")
        self.ik_service.wait_for_service()
        self.get_logger().info("Connected to IK service.")

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.pose_sub = self.create_subscription(Pose, "target_pose", self.pose_callback, qos)

    def pose_callback(self, pose):
        self.get_logger().info("Received Pose message, calling IK service...")
        request = InverseKinematics.Request()
        request.target_pose = pose

        future = self.ik_service.call_async(request)
        future.add_done_callback(self.service_result_handler)

    def service_result_handler(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info("IK solved successfully, publishing JointTrajectory.")
            jt_msg = JointTrajectory()
            jt_msg.joint_names = [f'joint_{i+1}' for i in range(len(result.joint_angles))]
            point = JointTrajectoryPoint()
            point.positions = result.joint_angles
            point.time_from_start.sec = 1
            jt_msg.points.append(point)
            self.joint_trajectory_pub.publish(jt_msg)
        else:
            self.get_logger().error("IK service failed")


def main(args=None):
    rclpy.init(args=args)
    node = PoseToJointTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
