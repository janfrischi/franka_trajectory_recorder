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

        # Initialize joint positions
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
        for i in range(7):
            joint_row_layout = QHBoxLayout()  # Layout for a single joint

            label = QLabel(f"Joint {i + 1}: 0.0")
            label.setFixedWidth(100)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-314)  # -3.14 radians scaled by 100
            slider.setMaximum(314)   # 3.14 radians scaled by 100
            slider.setValue(int(self.node.joint_positions[i] * 100))
            slider.valueChanged.connect(self.update_joint_position_from_slider(i, label))

            input_field = QLineEdit()
            input_field.setFixedWidth(80)
            input_field.setText(f"{self.node.joint_positions[i]:.2f}")
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
            # Convert slider value to radians
            position = value / 100.0
            self.node.joint_positions[joint_index] = position
            label.setText(f"Joint {joint_index + 1}: {position:.2f}")
            self.inputs[joint_index].setText(f"{position:.2f}")
        return callback

    def update_joint_position_from_input(self, joint_index, slider, label):
        def callback():
            try:
                # Get the value from the input field
                position = float(self.inputs[joint_index].text())
                if -3.14 <= position <= 3.14:
                    self.node.joint_positions[joint_index] = position
                    slider.setValue(int(position * 100))
                    label.setText(f"Joint {joint_index + 1}: {position:.2f}")
                else:
                    self.inputs[joint_index].setText(f"{self.node.joint_positions[joint_index]:.2f}")
            except ValueError:
                self.inputs[joint_index].setText(f"{self.node.joint_positions[joint_index]:.2f}")
        return callback

    def publish_joint_positions(self):
        self.node.publish_joint_positions()

    def reset_joint_positions(self):
        # Reset all sliders, labels, and input fields to the default joint positions
        for i in range(7):
            position = self.node.joint_positions[i]  # Default joint position
            self.sliders[i].setValue(int(position * 100))  # Reset slider
            self.labels[i].setText(f"Joint {i + 1}: {position:.2f}")  # Reset label
            self.inputs[i].setText(f"{position:.2f}")  # Reset input field

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