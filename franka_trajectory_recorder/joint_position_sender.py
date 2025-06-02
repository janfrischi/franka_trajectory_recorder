import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class JointPositionSender(Node):
    def __init__(self):
        super().__init__('joint_position_sender')

        # Declare parameters for mode and topic
        self.declare_parameter('mode', 'trajectory_playback')  # Options: 'trajectory_playback', 'policy_runner'
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('joint_positions', [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5])  # Default joint positions

        # Get parameters
        self.mode = self.get_parameter('mode').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.joint_positions = self.get_parameter('joint_positions').value

        # Determine the topic based on the mode
        if self.mode == 'trajectory_playback':
            self.topic = '/trajectory_playback/joint_positions'
        elif self.mode == 'policy_runner':
            self.topic = '/policy_outputs'
        else:
            self.get_logger().error(f"Invalid mode: {self.mode}. Use 'trajectory_playback' or 'policy_runner'.")
            rclpy.shutdown()
            return

        # Create publisher
        self.publisher = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.get_logger().info(f"Publishing joint positions to {self.topic} at {self.publish_rate} Hz.")

        # Create a timer to publish at the specified rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_positions)

    def publish_joint_positions(self):
        # Create the message
        msg = Float64MultiArray()
        msg.data = self.joint_positions

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint positions: {self.joint_positions}")

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Joint Position Command Sender.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()