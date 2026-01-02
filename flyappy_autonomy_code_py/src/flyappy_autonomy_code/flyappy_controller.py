import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point
import math
from typing import List, Dict, Any

class FlappyAutomation(Node):
    def __init__(self) -> None:
        super().__init__('controller_node')

        # Time constants
        self.FPS: float = 30.0
        self.T: float = 1.0 / self.FPS
        
        # Dynamics limits
        self.ACC_LIMIT: List[float] = [3.0, 10.0]
        self.VEL_LIMIT: float = 10.0
        
        # Proximity threshold
        self.TH_PROX_X: float = 0.1
        self.TH_PROX_Y: float = 0.04

        # Internal state
        self.bird_pos: List[float] = [0.0, 0.0]
        self.bird_vel: List[float] = [0.0, 0.0]
        
        # List of waypoints (Path)
        self.path: Dict[int, Dict[str, Any]] = {} 

        # Publisher & Subscriber
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
            
        self.sub_path = self.create_subscription(
            Point,
            '/flyappy_path',
            self.path_callback,
            10
        )

        self.get_logger().info("Control node started!")
        
    def minimum_braking_distance(self, v: float, a: float) -> float:
        """Calculates the distance required to come to a full stop"""
        if a == 0 or v == 0:
            return 0.0
        
        x: float = 0.0
        v_temp: float = abs(v)
        # Calculate the number of steps required to stop
        n: int = int(round(v_temp / (a * self.T)))
        if n == 0: return 0.0
        
        # Correct acceleration for discrete steps
        a_step: float = v_temp / (self.T * n)

        while v_temp > 0:
            v_temp -= a_step * self.T
            x += v_temp * self.T
        return x

    def path_callback(self, msg: Point) -> None:
        """Receives waypoints and stores them using Z as index"""
        idx: int = int(msg.z)
        self.path[idx] = {
            'pos': [msg.x, msg.y],
            'reached': False
        }

    def vel_callback(self, msg: Vector3) -> None:
        """Adjust the drone's position by integrating speed"""
        self.bird_pos[0] += msg.x * self.T
        self.bird_pos[1] += msg.y * self.T
        
        self.path_planner()
    
    def path_planner(self) -> None:
        """Calculates and publishes acceleration commands to reach waypoints"""
        acc_cmd: Vector3 = Vector3()
        
        # 1. TARGET IDENTIFICATION
        target_idx = None
        # Iterate over waypoints sorted by Z index
        for i in sorted(self.path.keys()):
            point: Dict[str, Any] = self.path[i]
            
            # Calculate relative distance between drone and waypoint
            dist_x: float = point['pos'][0] - self.bird_pos[0]
            dist_y: float = point['pos'][1] - self.bird_pos[1]
            
            if not point['reached']:
                # If within proximity threshold or if the X position has been passed
                if (abs(dist_x) < self.TH_PROX_X and abs(dist_y) < self.TH_PROX_Y) or \
                   (point['pos'][0] < self.bird_pos[0]):
                    point['reached'] = True
                    continue
                else:
                    target_idx = i
                    break
        
        # If there are no future points, maintain a minimum cruise speed for safety
        if target_idx is None:
            if self.bird_vel[0] < (self.VEL_LIMIT * 0.5):
                acc_cmd.x = self.ACC_LIMIT[0] - 1.5
            else:
                acc_cmd.x = 0.0
            acc_cmd.y = -self.bird_vel[1] / self.T # Attempt to stabilize Y at 0
            self.pub_acc.publish(acc_cmd)
            return

        target: Dict[str, Any] = self.path[target_idx]

        # 2. X-AXIS LOGIC
        dist_x = target['pos'][0] - self.bird_pos[0]
        # Distance required to brake at current speed
        curr_braking_x: float = self.minimum_braking_distance(self.bird_vel[0], self.ACC_LIMIT[0])

        if dist_x < self.TH_PROX_X:
            # STOP ZONE: slow down abruptly to stabilize on the point
            acc_cmd.x = -self.bird_vel[0] / self.T
        elif dist_x > curr_braking_x:
            # Enough space: accelerate up to speed limit
            acc_cmd.x = self.ACC_LIMIT[0] - 1.5
            if (self.bird_vel[0] + acc_cmd.x * self.T) > self.VEL_LIMIT:
                acc_cmd.x = (self.VEL_LIMIT - self.bird_vel[0]) / self.T
        else:
            # Must start braking to avoid missing the waypoint
            acc_cmd.x = -self.ACC_LIMIT[0]


        # 3. Y-AXIS LOGIC (GAP CENTERING)
        dist_y: float = target['pos'][1] - self.bird_pos[1]
        abs_dist_y: float = abs(dist_y)
        crit_y: float = self.minimum_braking_distance(self.VEL_LIMIT, self.ACC_LIMIT[1])
        curr_braking_y: float = self.minimum_braking_distance(self.bird_vel[1], self.ACC_LIMIT[1])

        if abs_dist_y < self.TH_PROX_Y:
            acc_cmd.y = -self.bird_vel[1] / self.T
        else:
            direction: float = 1.0 if dist_y > 0 else -1.0
            
            if abs_dist_y > crit_y:
                acc_cmd.y = direction * self.ACC_LIMIT[1]
                if abs(self.bird_vel[1] + acc_cmd.y * self.T) > self.VEL_LIMIT:
                    acc_cmd.y = (direction * self.VEL_LIMIT - self.bird_vel[1]) / self.T
            
            elif abs_dist_y <= abs(curr_braking_y):
                acc_cmd.y = -direction * self.ACC_LIMIT[1]
                if (direction > 0 and (self.bird_vel[1] + acc_cmd.y * self.T) < 0) or \
                   (direction < 0 and (self.bird_vel[1] + acc_cmd.y * self.T) > 0):
                    acc_cmd.y = (0.0 - self.bird_vel[1]) / self.T
            else:
                acc_cmd.y = direction * (self.ACC_LIMIT[1] * 0.5)

        #  4. PUBLISH COMMAND 
        self.pub_acc.publish(acc_cmd)
        
        self.get_logger().debug(f"Target [{target_idx}] - DistX: {dist_x:.2f}, DistY: {dist_y:.2f}, Acc: [{acc_cmd.x:.1f}, {acc_cmd.y:.1f}]")