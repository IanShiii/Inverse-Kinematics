import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from interfaces.srv import InverseKinematics


class PoseToJointTrajectoryNode(Node):
    def __init__(self):
        super().__init__('pose_to_joint_trajectory_node')

        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 1)

        # Wait for IK service
        self.get_logger().info("Waiting for IK service '%s'..." % self.ik_service_name)
        self.ik_service = self.create_client(InverseKinematics, self.ik_service_name)
        self.ik_service.wait_for_service()
        self.get_logger().info("Connected to IK service.")

        self.pose_sub = self.create_subscription(Pose, self.pose_topic, self.pose_callback, 1)

    def pose_callback(self, pose):
        self.get_logger().info("Received Pose message, calling IK service...")
        request = InverseKinematics.Request()
        request.pose = pose

        future = self.ik_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

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
            self.get_logger().error("IK service failed: %s" % result.message)


if __name__ == '__main__':
    rclpy.init()
    node = PoseToJointTrajectoryNode()
    rclpy.spin(node)
    rclpy.shutdown()
