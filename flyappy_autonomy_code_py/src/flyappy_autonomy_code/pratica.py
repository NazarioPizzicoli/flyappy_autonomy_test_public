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
    controller.get_logger().info("Controller avviato!")

    try:
        executor.spin()
    except KeyboardInterrupt:
        perception.get_logger().info("Arresto del sistema richiesto dall'utente...")
    finally:
        executor.shutdown()
        perception.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

########################################################################################
                                    #NODO PERCEPTION#
########################################################################################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3, Point
import math
from typing import List, Dict, Any, Union
 
class FlyappyPerception(Node):
    def __init__(self) -> None:
        super().__init__('perception_node')
        
        # PARAMETRI DA DEFINIRE
        self.FPS: float = 30.0
        self.T: float = 1.0 / self.FPS 
        
        self.lasers: List[Dict[str,Any]] = []
        
        # Space constraints
        self.RANGE_TH: float = -0.05
        self.X_TH: float = 1.4
        self.Y_TH: float = 0.50
        self.CEILING: float = 2.40
        self.FLOOR: float = -1.40
        self.REL_TH: float = 80.0
        
        # PUB
        self.pub_path = self.create_publisher(
            Point,
            '/flyappy_path',
            10
        )
        
        self.sub_vel = self.create_subscription(
            Vector3,
            '/flyappy_vel',
            self.vel_callback,
            10
        )
        
        self.sub_laser = self.create_subscription(
            LaserScan,
            '/flyappy_laser_scan',
            self.laser_scan_callback,
            10
        )
        
        self.get_logger().info("Perception node started!") #self.get_logger(). può essere in quanti modi?
    
    def vel_callback(self, msg:Vector3) -> None:
        self.bird_pos[0] += msg.x * self.T
        self.bird_pos[1] += msg.y * self.T
    
    def laser_scan_callback(self, msg: LaserScan) -> None:
        """Laser perception pipeline"""
        # 1. From polar to cartesian
        self.update_lasers(msg)
        
        # 2. Detect and initialize each column
        self.column_detector()
        
        # 3. Split space into gaps
        self.space_splitter()
        
        # 4. Rank the gaps
        self.split_ranker()
        
        # 5. Generate and publish the path
        self.path_finder()
        
    def update_lasers(self, msg:LaserScan) -> None:
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + (msg.angle_increment * i)
            pos_x = self.bird_pos[0] + msg.ranges[i] * math.cos(angle)
            pos_y = self.bird_pos[1] + msg.ranges[i] * math.sin(angle)
            
            self.lasers.append({
                'length' : msg.ranges[i],
                'pos': [pos_x,pos_y],
                'limit': msg.range_max - self.RANGE_TH    
            })
    
    def column_detector(self) -> None:
        for laser in self.laser:
            if (self.FLOOR < laser['pos'][1] < self.CEILING and
                laser['length'] < laser['limit']):
                
                if not self.columns:
                    self.add_new_column(laser['pos'][0])
                    continue
                
                found = False
                for col in self.columns:
                    if abs(laser['pos'][0] - self.column['x']) < self.X_TH:
                        found = True 
                        break
                if not found and laser['pos'][0] > (self.column[-1]['x'] + self.X_TH):
                    self.add_new_column(laser['pos'][0])
    
    def add_new_column(self, x_pos):
        new_col ={
            'x': x_pos,
            'point_max': [self.CEILING - self.Y_TH],
            'point_min': [self.FLOOR + self.Y_TH],
            'free': [-1],
            'score': [self.CEILING - self.FLOOR],
            'centre': [(self.CEILING - self.FLOOR) / 2],
            'reliability': [0.0],
            'best_idx': 0
        }
        self.columns.append(new_col)
        self.get_logger().info()