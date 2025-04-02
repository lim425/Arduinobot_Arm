#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class AngleConverter(Node):
    def __init__(self):
        super().__init__("angle_conversion_service_server")

        self.euler_to_quaternion_ = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.eulerToQuaternionCallback)
        self.quaternion_to_euler_ = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaternionToEulerCallback)
        self.get_logger().info("Angle Conversion Service is ready!")

    def eulerToQuaternionCallback(self, req, res):
        self.get_logger().info(f"Requested to convert euler angle roll:{req.roll}, pitch:{req.pitch}, yaw:{req.yaw}")
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info(f"Corresponding quaternion x:{res.x} y:{res.y} z:{res.z} w:{res.w}")
        return res

    def quaternionToEulerCallback(self, req, res):
        self.get_logger().info(f"Requested to convert quaternion x:{req.x}, y:{req.y}, z:{req.z}, w:{req.w}")
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        self.get_logger().info(f"Corresponding euler angle roll:{res.roll}, pitch:{res.pitch}, yaw:{res.yaw}")
        return res


def main():
    rclpy.init()
    node = AngleConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
