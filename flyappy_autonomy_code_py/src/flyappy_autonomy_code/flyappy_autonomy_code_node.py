#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from flyappy_autonomy_code.flyappy_perception import FlyappyPerception
from flyappy_autonomy_code.flyappy_controller import FlappyAutomation

def main(args=None):
    rclpy.init(args=args)

    perception = FlyappyPerception()
    controller = FlappyAutomation()

    executor = MultiThreadedExecutor()

    executor.add_node(perception)
    executor.add_node(controller)

    perception.get_logger().info("Perception node started!")
    controller.get_logger().info("Control node started!")

    try:
        executor.spin()
    except KeyboardInterrupt:
        perception.get_logger().info("Keyboard interrupt request...")
    finally:
        # Pulizia sistematica
        executor.shutdown()
        perception.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()