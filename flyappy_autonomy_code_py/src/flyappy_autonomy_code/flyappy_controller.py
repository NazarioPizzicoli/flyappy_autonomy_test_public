import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point
import math
from typing import List, Dict, Any

class FlappyAutomation(Node):
    def __init__(self) -> None:
        super().__init__('controller_node')

        # Control constants
        self.FPS: float = 30.0
        self.T: float = 1.0 / self.FPS
        
        # Dynamics limits [X, Y]
        self.ACC_LIMIT = [3.0, 10.0]
        self.VEL_LIMIT = 10.0
        
        # Proximity threshold
        self.TH_PROX = [0.1, 0.04]

        # Internal state
        self.pos_bird = [0.0, 0.0]
        self.vel_bird = [0.0, 0.0]
        self.path = {} 

        # Publishers and Subscribers
        self.pub_acc = self.create_publisher(
            Vector3,
            '/flyappy_acc',
            10
        )
        
        self.sub_vel = self.create_subscription(
            Vector3,
            '/flyappy_vel',
            self.vel_callback,
            10
         )
        
        self.sub_path = self.create_subscription(Point,
            '/flyappy_path',
            self.path_callback,
            10
        )

    def vel_callback(self, msg: Vector3) -> None:
        """Update bird position and trigger path planning logic"""
        self.vel_bird = [msg.x, msg.y]
        self.pos_bird[0] += self.vel_bird[0] * self.T
        self.pos_bird[1] += self.vel_bird[1] * self.T
        self.path_planner()
    
    def path_callback(self, msg: Point) -> None:
        """Store incoming waypoints using Z coordinate as the index"""
        idx: int = int(msg.z)
        self.path[idx] = {
            'pos': [msg.x, msg.y],
            'reached': False
        }
    
    def get_braking_dist(self, vel: float, acc: float) -> float:
        """Calculate the distance needed to stop based on current speed and acceleration"""
        if acc == 0 or vel == 0:
            return 0.0
        
        dist= 0.0
        vel_temp = abs(vel)
        n_steps: int = round(vel_temp / (acc * self.T))
        
        if n_steps == 0: return 0.0
        
        acc_step = vel_temp / (self.T * n_steps)
        while vel_temp > 0:
            vel_temp -= acc_step * self.T
            dist += vel_temp * self.T
        return dist
     
    def get_next_target(self):
        """Identify the next available waypoint in the path"""
        for i in sorted(self.path.keys()):
            point = self.path[i]
            if point['reached']: continue
            
            dist_x = point['pos'][0] - self.pos_bird[0]
            dist_y = point['pos'][1] - self.pos_bird[1]
            
            # Check if current waypoint is reached or passed
            if (abs(dist_x) < self.TH_PROX[0] and abs(dist_y) < self.TH_PROX[1]) or \
                (point['pos'][0] < self.pos_bird[0]):
                    point['reached'] = True
                    continue
            return i, point
        return None, None
                             
    def compute_acc_x(self, target_x: float) -> float:
        """Calculate acceleration/deceleration for the X axis"""
        dist_x = target_x - self.pos_bird[0]
        stop_dist = self.get_braking_dist(self.vel_bird[0], self.ACC_LIMIT[0])

        if dist_x < self.TH_PROX[0]:
            return -self.vel_bird[0] / self.T  # Stabilize on point
        elif dist_x > stop_dist:
            # Safe to accelerate towards target
            acc = self.ACC_LIMIT[0] - 1.5
            if (self.vel_bird[0] + acc * self.T) > self.VEL_LIMIT:
                acc = (self.VEL_LIMIT - self.vel_bird[0]) / self.T
            return acc
        else:
            return -self.ACC_LIMIT[0] # Decelerate to avoid overshoot

    def compute_acc_y(self, target_y: float) -> float:
        """Calculate acceleration for the Y axis using critical braking logic"""
        dist_y = target_y - self.pos_bird[1]
        abs_dist_y = abs(dist_y)
        
        # Calculate thresholds for switching between acceleration and braking
        crit_dist = self.get_braking_dist(self.VEL_LIMIT, self.ACC_LIMIT[1]) 
        actual_brak = self.get_braking_dist(self.vel_bird[1], self.ACC_LIMIT[1])

        if abs_dist_y < self.TH_PROX[1]:
            return -self.vel_bird[1] / self.T
        
        direction = 1.0 if dist_y > 0 else -1.0
        
        if abs_dist_y > crit_dist:
            # Full speed ahead towards the gap center
            acc = direction * self.ACC_LIMIT[1]
            if abs(self.vel_bird[1] + acc * self.T) > self.VEL_LIMIT:
                acc = (direction * self.VEL_LIMIT - self.vel_bird[1]) / self.T
            return acc
        elif abs_dist_y <= abs(actual_brak):
            # Critical braking: counter-thrust to align with the gap
            acc = -direction * self.ACC_LIMIT[1]
            if (direction > 0 and (self.vel_bird[1] + acc * self.T) < 0) or \
               (direction < 0 and (self.vel_bird[1] + acc * self.T) > 0):
                acc = (0.0 - self.vel_bird[1]) / self.T
            return acc
        else:
            return direction * (self.ACC_LIMIT[1] * 0.5) # Approaching gap
    
    def path_planner(self) -> None:
        """Main planning loop to calculate and publish acceleration commands"""
        idx, target = self.get_next_target()
        acc_cmd = Vector3()

        if target is not None:
            acc_cmd.x = self.compute_acc_x(target['pos'][0])
            acc_cmd.y = self.compute_acc_y(target['pos'][1])
            self.get_logger().debug(f"Targeting target [{idx}]")
        else:
            # No path left: maintain speed and level flight
            if self.vel_bird[0] < (self.VEL_LIMIT * 0.5):
                acc_cmd.x = self.ACC_LIMIT[0] - 1.5
            acc_cmd.y = -self.vel_bird[1] / self.T
            
        self.pub_acc.publish(acc_cmd)