import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.1


class CircleMotion(Node):
    def __init__(self):
        super().__init__("circle_motion")
        self.ready = not self.declare_parameter("wait_for_amcl", False).value
        if not self.ready:
            self.pose_subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                "/amcl_pose",
                self.start,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
        self.publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def start(self, _message):
        self.ready = True

    def publish_velocity(self):
        if not self.ready:
            return
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        command.twist.linear.x = LINEAR_SPEED
        command.twist.angular.z = ANGULAR_SPEED
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = CircleMotion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
