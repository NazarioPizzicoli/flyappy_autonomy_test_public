import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Vector3
import math
from typing import List, Dict, Any
from dataclasses import dataclass

@dataclass
class Gap:
    """Represents a navigable opening between obstacles"""
    upper_limit: float
    bottom_limit: float
    free: bool = True
    rel:float = 0.0
    
    @property
    def center(self) -> float:
        return (self.upper_limit + self.bottom_limit) / 2
    
    @property
    def height(self) -> float:
        return self.upper_limit - self.bottom_limit
    
class FlyappyPerception(Node):
    def __init__(self) -> None:
        super().__init__('perception_node')

        # Environment constants
        self.FPS: float = 30.0
        self.T: float = 1.0 / self.FPS
        
        # Geometry and Thresholds
        self.MAX_RANGE: float = 3.5     #(3.549 -0.050)
        self.X_TH_COLUMN: float = 1.4   # Minimum distance between column
        self.Y_TH_GAP: float = 0.50     # Minimum height to be a gap
        self.CEILING: float = 2.40
        self.FLOOR: float = -1.40
        self.REL_TH: float = 80.0       # Reliability threshold for fast entry
        
        # Pathfinding offsets
        self.offset_entry_far: float = 1.2 
        self.offset_entry_near: float = 0.6
        self.offset_exit: float = 0.8

        # State tracking
        self.pos_bird = [0.0, 0.0]
        self.lasers = []
        self.columns = []
        
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
            
    def vel_callback(self, msg: Vector3) -> None:
        """Adjust the drone's position by integrating speed"""
        self.pos_bird[0] += msg.x * self.T
        self.pos_bird[1] += msg.y * self.T
        
    def laser_scan_callback(self, msg: LaserScan) -> None:
        """Laser perception pipeline"""
        self.update_lasers(msg)
        self.column_detector()
        self.space_splitter()
        self.split_ranker()
        self.path_finder()

    def update_lasers(self, msg: LaserScan) -> None:
        """Transform laser readings from polar to global Cartesian coordinates"""
        self.lasers = []       
        for i, dist in enumerate(msg.ranges):
            angle: float = msg.angle_min + (msg.angle_increment * i)
            pos_x: float = self.pos_bird[0] + dist * math.cos(angle)
            pos_y: float = self.pos_bird[1] + dist * math.sin(angle)
            self.lasers.append({
                'length': dist,
                'pos': [pos_x, pos_y],
                'limit': self.MAX_RANGE
            })

    def column_detector(self) -> None:
        """Identify distinct vertical obstacles (columns)"""
        for laser in self.lasers:
            pos_x, pos_y = laser['pos']
            if (self.FLOOR < pos_y < self.CEILING and laser['length'] < laser['limit']):
                
                if not self.columns:
                    self.add_new_column(pos_x)
                    continue

                found = any((abs(pos_x - col['x']) < self.X_TH_COLUMN for col in self.columns))
                
                if not found:
                    if not self.columns or pos_x > (self.columns[-1]['x'] + self.X_TH_COLUMN):
                        self.add_new_column(pos_x)

    def add_new_column(self, x_pos: float) -> None:
        """Build data structures for a new column"""
        gap_initial = Gap(upper_limit=self.CEILING, bottom_limit=self.FLOOR)
        new_col = {
            'x': x_pos,
            'gaps': [gap_initial],
            'best_idx': 0
        }
        self.columns.append(new_col)
        self.get_logger().info(f"Detected column {len(self.columns)-1} at X: {x_pos:.2f}")

    def space_splitter(self) -> None:
        """Split columns into gaps"""
        for laser in self.lasers:
            pos_x, pos_y = laser['pos']
            if not (self.FLOOR < pos_y < self.CEILING and #PALESE SBAGLIATO
                laser['length'] < laser['limit']):
                    continue
                
            for col in self.columns:
                if abs(pos_x - col['x']) < self.X_TH_COLUMN:
                    for gap in col['gaps']:
                        # If laser hits within a free gap, the gap is split by the obstacles
                        if gap.free and gap.bottom_limit < pos_y < gap.upper_limit:
                            old_max = gap.upper_limit
                            
                            # Update current gap to be the lower section
                            gap.upper_limit = pos_y
                            if gap.height < self.Y_TH_GAP: gap.free = False
                            
                            # Create a new gap for the upper section
                            new = Gap(upper_limit=old_max, bottom_limit=pos_y)
                            if new.height < self.Y_TH_GAP: new.free = False
                            
                            col['gaps'].append(new)
                            break

    def split_ranker(self) -> None:
        """Selects the best gap and prints the required test data"""
        for col in self.columns:
            if self.pos_bird[0] > col['x']: continue
            
            free_gaps = [gap for gap in col['gaps'] if gap.free]
            total_height = sum (gap.height for gap in free_gaps)
            
            if total_height <= 0: continue

            best_rel = -1.0
            for i, gap in enumerate(col['gaps']):
                if gap.free:
                    # Score based on size relative to total space and required height
                    rel = (gap.height / total_height) * (self.Y_TH_GAP / gap.height)**2 * 100
                    gap.rel = rel
                    if rel > best_rel:
                        best_rel = rel
                        col['best_idx'] = i
                        
    def path_finder(self) -> None:
        """Calculate and publish waypoints (entrance/exit) for the best gaps in each column"""
        for i, col in enumerate(self.columns):
            best = col['gaps'][col['best_idx']]
            
            # Select entrance offset based on the reliability of the detection
            offset = self.offset_entry_far if best.rel <= self.REL_TH else self.offset_entry_near
            
            # Entrance point
            point_in = Point(x=col['x'] - offset, y=best.center , z=float(i*2))
            if self.pos_bird[0] <= point_in.x: self.pub_path.publish(point_in)
            
            # Exit point
            point_out = Point(x=col['x'] + self.offset_exit, y=best.center, z=float(i*2+1))
            if self.pos_bird[0] <= point_out.x: self.pub_path.publish(point_out)
            
