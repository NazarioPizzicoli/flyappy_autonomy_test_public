#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from flyappy_autonomy_code.flyappy_perception import FlyappyPerception
from flyappy_autonomy_code.flyappy_controller import FlappyAutomation

def main(args=None):
    rclpy.init(args=args)

    # Inizializziamo le due classi (nodi)
    perception = FlyappyPerception()
    controller = FlappyAutomation()

    # Usiamo un MultiThreadedExecutor per permettere ai callback
    # di entrambi i nodi di girare in parallelo senza bloccarsi
    executor = MultiThreadedExecutor()

    # Aggiungiamo i nodi all'executor
    executor.add_node(perception)
    executor.add_node(controller)

    perception.get_logger().info("Perception node started!")
    controller.get_logger().info("Controller avviato!")

    try:
        # Avviamo il loop principale che gestisce entrambi i nodi
        executor.spin()
    except KeyboardInterrupt:
        perception.get_logger().info("Arresto del sistema richiesto dall'utente...")
    finally:
        # Pulizia sistematica
        executor.shutdown()
        perception.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()