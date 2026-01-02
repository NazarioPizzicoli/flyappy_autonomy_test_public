import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Vector3
import math
from typing import List, Dict, Any

class FlyappyPerception(Node):
    def __init__(self) -> None:
        super().__init__('perception_node')

        # Time constants
        self.FPS: float = 30.0
        self.T: float = 1.0 / self.FPS
        
        # Space constraints
        self.RANGE_TH: float = -0.05
        self.X_TH: float = 1.4
        self.Y_TH: float = 0.50
        self.CEILING: float = 2.40
        self.FLOOR: float = -1.40
        self.REL_TH: float = 80.0
        
        # Thresholds for path calculation
        self.offset_entry_far: float = 1.2
        self.offset_entry_near: float = 0.6
        self.offset_exit: float = 0.8

        # Internal state
        self.bird_pos: List[float] = [0.0, 0.0]
        
        # Laser and column list of dict
        self.lasers: List[Dict[str, Any]] = []
        self.columns: List[Dict[str, Any]] = []

        # Publisher & Subscriber
        self.pub_path = self.create_publisher(
            Point,
            '/flyappy_path',
            10 # da provare con QOS uguale ad 1
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
            
        self.get_logger().info("Perception node started!")

    def vel_callback(self, msg: Vector3) -> None:
        """Adjust the drone's position by integrating speed"""
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

    def update_lasers(self, msg: LaserScan) -> None:
        """Convert information from polar coordinates to cartesian coordinates"""
        # self.lasers = []

        for i in range(len(msg.ranges)):
            angle: float = msg.angle_min + (msg.angle_increment * i)
            
            pos_x: float = self.bird_pos[0] + msg.ranges[i] * math.cos(angle)
            pos_y: float = self.bird_pos[1] + msg.ranges[i] * math.sin(angle)
            
            self.lasers.append({
                'length': msg.ranges[i],
                'pos': [pos_x, pos_y],
                'limit': msg.range_max + self.RANGE_TH
            })

    def column_detector(self) -> None:
        """Detect and initialize each column"""
        for laser in self.lasers:
            # Laser conditions
            if (self.FLOOR < laser['pos'][1] < self.CEILING and 
                laser['length'] < laser['limit']):
                                
                if not self.columns:
                    self.add_new_column(laser['pos'][0])
                    continue

                found: bool = False
                for col in self.columns:
                    if abs(laser['pos'][0] - col['x']) < self.X_TH:
                        found = True
                        break
                
                if not found and laser['pos'][0] > (self.columns[-1]['x'] + self.X_TH):
                    self.add_new_column(laser['pos'][0])

    def add_new_column(self, x_pos: float) -> None:
        """Build data structures for a new column"""
        new_col: Dict[str, Any] = {
            'x': x_pos,
            'point_max': [self.CEILING - self.Y_TH],
            'point_min': [self.FLOOR + self.Y_TH],
            'free': [-1], #-1 gap(free), 1 wall(full)
            'score': [self.CEILING - self.FLOOR],
            'center': [(self.CEILING + self.FLOOR) / 2],
            'reliability': [0.0],
            'best_idx': 0
        }
        self.columns.append(new_col)
        self.get_logger().info(f"Detected column {len(self.columns)-1} at X: {x_pos:.2f}")

    def space_splitter(self) -> None:
        """Split columns into gaps"""
        for laser in self.lasers:
            if (self.FLOOR < laser['pos'][1] < self.CEILING and 
                laser['length'] < laser['limit']):

                for col in self.columns:
                    if abs(laser['pos'][0] - col['x']) < self.X_TH:
                        for m in range(len(col['free'])):
                            if col['free'][m] == -1 and col['point_min'][m] < laser['pos'][1] < col['point_max'][m]:
                                old_max: float = col['point_max'][m]
                                
                                # Split current gap (lower part)
                                col['point_max'][m] = laser['pos'][1]
                                col['score'][m] = col['point_max'][m] - col['point_min'][m]
                                col['center'][m] = (col['point_max'][m] + col['point_min'][m]) / 2
                                if abs(col['score'][m]) < self.Y_TH: col['free'][m] = 1
                                
                                # Add new gap (upper part)
                                col['point_max'].append(old_max)
                                col['point_min'].append(laser['pos'][1])
                                col['free'].append(-1)
                                col['score'].append(old_max - laser['pos'][1])
                                col['center'].append((old_max + laser['pos'][1]) / 2)
                                col['reliability'].append(0.0)
                                if abs(col['score'][-1]) < self.Y_TH: col['free'][-1] = 1
                                break

    def split_ranker(self) -> None:
        """Selects the best gap and prints the required test data"""
        for i, col in enumerate(self.columns):
            if self.bird_pos[0] > col['x']: continue
            
            score_sum: float = sum(col['score'][m] for m in range(len(col['free'])) if col['free'][m] == -1)
            if score_sum <= 0: continue

            best_rel: float = -1.0
            for m in range(len(col['free'])):
                if col['free'][m] == -1:
                    rel: float = (col['score'][m] / score_sum) * (self.Y_TH / col['score'][m])**2 * 100
                    col['reliability'][m] = rel
                    if rel > best_rel:
                        best_rel = rel
                        col['best_idx'] = m

            best_m: int = col['best_idx']
            gap_center:float = col['center'][best_m]
            diff_y:float = gap_center - self.bird_pos[1]

    def path_finder(self) -> None:
        """Publishes waypoints for drone control"""
        for i, col in enumerate(self.columns):
            best_m: int = col['best_idx']
            offset: float = self.offset_entry_far if col['reliability'][best_m] <= self.REL_TH else self.offset_entry_near
            
            # Entrance point
            point_in = Point(x=col['x']-offset, y=col['center'][best_m], z=float(i*2))
            if self.bird_pos[0] <= point_in.x: self.pub_path.publish(point_in)
            
            # Exit point
            point_out = Point(x=col['x']+self.offset_exit, y=col['center'][best_m], z=float(i*2+1))
            if self.bird_pos[0] <= point_out.x: self.pub_path.publish(point_out)