#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DualArmJointStateBridge(Node):
    def __init__(self):
        super().__init__("dual_arm_joint_state_bridge")

        self.declare_parameter("left_topic", "/left_arm/joint_states")
        self.declare_parameter("right_topic", "/right_arm/joint_states")
        self.declare_parameter("output_topic", "/joint_states")

        self._left_topic = self.get_parameter("left_topic").value
        self._right_topic = self.get_parameter("right_topic").value
        self._output_topic = self.get_parameter("output_topic").value

        self._latest = {"left": None, "right": None}

        self._publisher = self.create_publisher(JointState, self._output_topic, 10)
        self.create_subscription(JointState, self._left_topic, self._left_callback, 10)
        self.create_subscription(JointState, self._right_topic, self._right_callback, 10)

        self.get_logger().info(
            f"Bridging '{self._left_topic}' + '{self._right_topic}' -> '{self._output_topic}'"
        )

    def _left_callback(self, msg: JointState):
        self._latest["left"] = msg
        self._publish_merged()

    def _right_callback(self, msg: JointState):
        self._latest["right"] = msg
        self._publish_merged()

    def _publish_merged(self):
        msgs = [self._latest["left"], self._latest["right"]]
        msgs = [msg for msg in msgs if msg is not None and msg.name]
        if not msgs:
            return

        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()

        include_position = any(len(msg.position) == len(msg.name) for msg in msgs)
        include_velocity = any(len(msg.velocity) == len(msg.name) for msg in msgs)
        include_effort = any(len(msg.effort) == len(msg.name) for msg in msgs)

        for msg in msgs:
            joint_count = len(msg.name)
            merged.name.extend(msg.name)

            if include_position:
                if len(msg.position) == joint_count:
                    merged.position.extend(msg.position)
                else:
                    merged.position.extend([0.0] * joint_count)

            if include_velocity:
                if len(msg.velocity) == joint_count:
                    merged.velocity.extend(msg.velocity)
                else:
                    merged.velocity.extend([0.0] * joint_count)

            if include_effort:
                if len(msg.effort) == joint_count:
                    merged.effort.extend(msg.effort)
                else:
                    merged.effort.extend([0.0] * joint_count)

        self._publisher.publish(merged)


def main():
    rclpy.init()
    node = DualArmJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
