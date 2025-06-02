import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import (
    QApplication, QSlider, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QLineEdit, QPushButton, QGroupBox
)
from PyQt5.QtCore import Qt
import sys
import numpy as np


class JointPositionSender(Node):
    def __init__(self, topic, publish_rate):
        super().__init__('joint_position_sender')

        # Initialize joint positions to default values
        self.joint_positions = np.array([0.0444, -0.1894, -0.1107, -2.5148, 0.0044, 2.3775, 0.6952])

        # Create publisher
        self.publisher = self.create_publisher(Float64MultiArray, topic, 10)
        self.get_logger().info(f"Publishing joint positions to {topic} at {publish_rate} Hz.")

        # Create a timer to publish at the specified rate
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_joint_positions)

    def publish_joint_positions(self):
        # Create the message
        msg = Float64MultiArray()
        msg.data = self.joint_positions.tolist()  # Convert NumPy array to Python list

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint positions: {self.joint_positions}")


class JointPositionGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # Set up the GUI layout
        self.setWindowTitle("Joint Position Controller")
        main_layout = QVBoxLayout()

        # Add a title
        title = QLabel("Franka Joint Position Controller")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        main_layout.addWidget(title)

        # Add a group box for joint controls
        joint_group = QGroupBox("Joint Controls")
        joint_layout = QVBoxLayout()  # Main layout for all joints

        self.sliders = []
        self.labels = []
        self.inputs = []

        # Create sliders, labels, and input fields for each joint
        # Joint limits for Franka FR3 robot (in radians)
        joint_limits = [
            (-2.7437, 2.7437),   # Joint 1
            (-1.7837, 1.7837),   # Joint 2
            (-2.9007, 2.9007),   # Joint 3
            (-3.0421, -0.1518),  # Joint 4
            (-2.8065, 2.8065),   # Joint 5
            (0.5445, 4.5169),    # Joint 6
            (-3.0159, 3.0159)    # Joint 7
        ]
        
        for i in range(7):
            joint_row_layout = QHBoxLayout()  # Layout for a single joint

            label = QLabel(f"Joint {i + 1}: 0.0")
            label.setFixedWidth(100)

            slider = QSlider(Qt.Horizontal)
            # Set slider limits based on actual joint limits (scaled by 1000 for better precision)
            lower_limit, upper_limit = joint_limits[i]
            slider.setMinimum(int(lower_limit * 1000))
            slider.setMaximum(int(upper_limit * 1000))
            
            # Clamp initial position to joint limits
            initial_pos = max(lower_limit, min(upper_limit, self.node.joint_positions[i]))
            slider.setValue(int(initial_pos * 1000))
            slider.valueChanged.connect(self.update_joint_position_from_slider(i, label))

            input_field = QLineEdit()
            input_field.setFixedWidth(80)
            input_field.setText(f"{initial_pos:.3f}")
            input_field.editingFinished.connect(self.update_joint_position_from_input(i, slider, label))

            self.labels.append(label)
            self.sliders.append(slider)
            self.inputs.append(input_field)

            joint_row_layout.addWidget(label)
            joint_row_layout.addWidget(slider)
            joint_row_layout.addWidget(input_field)

            joint_layout.addLayout(joint_row_layout)  # Add the row layout to the main joint layout

        joint_group.setLayout(joint_layout)
        main_layout.addWidget(joint_group)

        # Add a publish button
        publish_button = QPushButton("Publish Joint Positions")
        publish_button.clicked.connect(self.publish_joint_positions)
        main_layout.addWidget(publish_button)

        # Add a reset button
        reset_button = QPushButton("Reset to Default")
        reset_button.clicked.connect(self.reset_joint_positions)
        main_layout.addWidget(reset_button)

        self.setLayout(main_layout)

    def update_joint_position_from_slider(self, joint_index, label):
        def callback(value):
            # Convert slider value to radians (scaled by 1000)
            position = value / 1000.0
            self.node.joint_positions[joint_index] = position
            label.setText(f"Joint {joint_index + 1}: {position:.3f}")
            self.inputs[joint_index].setText(f"{position:.3f}")
        return callback

    def update_joint_position_from_input(self, joint_index, slider, label):
        # Joint limits for validation
        joint_limits = [
            (-2.7437, 2.7437),   # Joint 1
            (-1.7837, 1.7837),   # Joint 2
            (-2.9007, 2.9007),   # Joint 3
            (-3.0421, -0.1518),  # Joint 4
            (-2.8065, 2.8065),   # Joint 5
            (0.5445, 4.5169),    # Joint 6
            (-3.0159, 3.0159)    # Joint 7
        ]
        
        def callback():
            try:
                # Get the value from the input field
                position = float(self.inputs[joint_index].text())
                lower_limit, upper_limit = joint_limits[joint_index]
                
                if lower_limit <= position <= upper_limit:
                    self.node.joint_positions[joint_index] = position
                    slider.setValue(int(position * 1000))
                    label.setText(f"Joint {joint_index + 1}: {position:.3f}")
                else:
                    # Reset to current valid value if out of range
                    current_pos = self.node.joint_positions[joint_index]
                    self.inputs[joint_index].setText(f"{current_pos:.3f}")
                    self.node.get_logger().warning(f"Joint {joint_index + 1} position {position:.3f} is out of range [{lower_limit:.3f}, {upper_limit:.3f}]")
            except ValueError:
                # Reset to current valid value if invalid input
                current_pos = self.node.joint_positions[joint_index]
                self.inputs[joint_index].setText(f"{current_pos:.3f}")
        return callback

    def publish_joint_positions(self):
        self.node.publish_joint_positions()

    def reset_joint_positions(self):
        # Joint limits for clamping
        joint_limits = [
            (-2.7437, 2.7437),   # Joint 1
            (-1.7837, 1.7837),   # Joint 2
            (-2.9007, 2.9007),   # Joint 3
            (-3.0421, -0.1518),  # Joint 4
            (-2.8065, 2.8065),   # Joint 5
            (0.5445, 4.5169),    # Joint 6
            (-3.0159, 3.0159)    # Joint 7
        ]
        
        # Reset all sliders, labels, and input fields to the default joint positions
        for i in range(7):
            # Clamp position to joint limits
            lower_limit, upper_limit = joint_limits[i]
            position = max(lower_limit, min(upper_limit, self.node.joint_positions[i]))
            
            self.sliders[i].setValue(int(position * 1000))  # Reset slider
            self.labels[i].setText(f"Joint {i + 1}: {position:.3f}")  # Reset label
            self.inputs[i].setText(f"{position:.3f}")  # Reset input field

        # Publish the reset joint positions
        self.node.publish_joint_positions()


def main(args=None):
    rclpy.init(args=args)

    # ROS 2 parameters
    topic = '/trajectory_playback/joint_positions'  # Change to '/policy_outputs' for policy runner mode
    publish_rate = 10.0  # Hz

    # Create the ROS 2 node
    node = JointPositionSender(topic, publish_rate)

    # Create the Qt application
    app = QApplication(sys.argv)
    gui = JointPositionGUI(node)
    gui.show()

    # Run the ROS 2 node and Qt application in parallel
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            app.processEvents()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Joint Position Command Sender.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()