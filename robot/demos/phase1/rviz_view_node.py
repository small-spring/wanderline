from rclpy.node import Node

import math
import numpy as np
from scipy.spatial.transform import Rotation as R

# Import necessary message types
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from phase1_robot_drawing.msg import PenState 
# /phase1/msgs/PenState.msg, 
# contains: Point tip_position, Point base_position, bool is_contact 

class RvizViewNode(Node):
    def __init__(self):
        super().__init__('rviz_view_node')

        # SUBSCRIBERS:
        # pen_state_sub: Subscribe to pen state updates -> pen_state_callback
        self.pen_state_sub = self.create_subscription(
            PenState,
            '/pen_state',
            self.pen_state_callback,
            10
        )

        # PUBLISHERS:
        # canvas_pub: Canvas visualization -> publish_canvas
        self.canvas_pub = self.create_publisher(Marker, '/canvas_marker', 10)
        # pen_trail_pub: Pen trail visualization -> add_pen_trail_point
        self.pen_trail_pub = self.create_publisher(MarkerArray, '/pen_trail', 10)
        # pen_body_pub: Pen body visualization
        self.pen_body_pub = self.create_publisher(Marker, '/pen_body', 10)
        # pen_tip_pub: Pen tip visualization
        self.pen_tip_pub = self.create_publisher(Marker, '/pen_tip', 10)

        # Internal state
        self.pen_trail_points = []


    def pen_state_callback(self, msg: PenState):
        """Handle incoming pen state (tip, base, contact)."""
        ## Unwrap message:
        # Extract tip and base positions
        tip_pos = [msg.tip_position.x, msg.tip_position.y, msg.tip_position.z]
        base_pos = [msg.base_position.x, msg.base_position.y, msg.base_position.z]

        # Store current pen state
        self.current_tip_pos = tip_pos
        self.current_base_pos = base_pos
        self.is_contact = msg.is_contact

        # Store canvas position
        self.canvas_position = [0.6, 0.0, 0.05]  # todo: load from config or set dynamically

        ## Publishing for Rviz:
        # Only add to trail and update tip when pen is in contact
        if msg.is_contact:
            self.add_pen_trail_point(tip_pos)
            self.publish_pen_tip(tip_pos)
        else:
            self.hide_pen_tip()  # Hide pen tip when not in contact

        # Show always: pen body and canvas, trail
        self.publish_pen_body()
        self.publish_canvas() 
        self.publish_pen_trail()


    def hide_pen_tip(self):
        """Hide the pen tip by publishing an empty marker."""
        empty_marker = Marker()
        empty_marker.header.frame_id = "base_link"
        empty_marker.header.stamp = self.get_clock().now().to_msg()
        empty_marker.ns = "pen_tip"
        empty_marker.id = 0
        empty_marker.action = Marker.DELETE

        self.pen_tip_pub.publish(empty_marker)

    def publish_canvas(self):
        """Publish a canvas marker in RViz."""
        canvas_marker = Marker()

        # Canvas marker properties
        canvas_marker.header.frame_id = "base_link"
        canvas_marker.header.stamp = self.get_clock().now().to_msg() # do we need this?
        canvas_marker.ns = "canvas"
        canvas_marker.id = 0
        canvas_marker.type = Marker.CUBE
        canvas_marker.action = Marker.ADD

        # Canvas position
        canvas_marker.pose.position.x = self.canvas_position[0]
        canvas_marker.pose.position.y = self.canvas_position[1]
        canvas_marker.pose.position.z = self.canvas_position[2]
        canvas_marker.pose.orientation.w = 1.0

        # Canvas size
        canvas_marker.scale.x = 0.4 # m related to: CANVAS_PHYSICAL_SIZE, todo: make it configurable
        canvas_marker.scale.y = 0.4 # m related to: CANVAS_PHYSICAL_SIZE
        canvas_marker.scale.z = 0.005 # m thickness

        # White canvas color, semi-transparent
        canvas_marker.color.r = 1.0
        canvas_marker.color.g = 1.0
        canvas_marker.color.b = 1.0
        canvas_marker.color.a = 0.9 # mostly opaque

        # Lifetime
        canvas_marker.lifetime.sec = 0
        canvas_marker.lifetime.nanosec = 0

        # Publish the canvas marker
        self.canvas_pub.publish(canvas_marker)

    def add_pen_trail_point(self, pos): # todo: trailにaddすることと、publishすることを分離する。
        # call service to add a point to the pen trail

        self.pen_trail_points.append(pos)
        if len(self.pen_trail_points) > 5000:
            self.pen_trail_points.pop(0)

        if len(self.pen_trail_points) % 10 == 0:
            self.publish_pen_trail()

    def publish_pen_trail(self):
        """Publish pen trail as line strip marker in RViz."""
        if len(self.pen_trail_points) < 2:
            return # Not enough points to draw trail

        ##### trail_marker: continuous red line #####
        # Create line strip marker for pen trail
        trail_marker = Marker()
        trail_marker.header.frame_id = "base_link"
        trail_marker.header.stamp = self.get_clock().now().to_msg()
        trail_marker.ns = "pen_trail"
        trail_marker.id = 0
        trail_marker.type = Marker.LINE_STRIP # -> continuous line
        trail_marker.action = Marker.ADD
        
        # Set line properties
        trail_marker.scale.x = 0.002  # 2mm line width
        trail_marker.color.r = 1.0    # Red color
        trail_marker.color.g = 0.0
        trail_marker.color.b = 0.0
        trail_marker.color.a = 0.8    # Semi-transparent
        
        # IMPORTANT: Set lifetime to persistent (prevent auto-deletion)
        trail_marker.lifetime.sec = 0
        trail_marker.lifetime.nanosec = 0
        
        # Add all points to the line strip - use actual robot positions
        for pos in self.pen_trail_points:
            point = Point()
            point.x = pos[0]
            point.y = pos[1] 
            point.z = pos[2]  # Use actual Z position for accurate representation
            trail_marker.points.append(point)
        
        ##### current_marker: current pen position, green sphere #####
        # Current pen position marker
        current_marker = Marker()
        current_marker.header.frame_id = "base_link"
        current_marker.header.stamp = self.get_clock().now().to_msg()
        current_marker.ns = "pen_current"
        current_marker.id = 1
        current_marker.type = Marker.SPHERE
        current_marker.action = Marker.ADD
        
        # Set current pen position - use actual robot position
        current_pos = self.pen_trail_points[-1]
        current_marker.pose.position.x = current_pos[0]
        current_marker.pose.position.y = current_pos[1]
        current_marker.pose.position.z = current_pos[2]  # Use actual Z position
        current_marker.pose.orientation.w = 1.0
        
        # Set sphere properties - smaller pen tip
        current_marker.scale.x = 0.005  # 5mm diameter
        current_marker.scale.y = 0.005
        current_marker.scale.z = 0.005
        current_marker.color.r = 0.0
        current_marker.color.g = 1.0  # Green color
        current_marker.color.b = 0.0
        current_marker.color.a = 1.0
        
        # IMPORTANT: Set lifetime to persistent (prevent auto-deletion)
        current_marker.lifetime.sec = 0
        current_marker.lifetime.nanosec = 0
        
        ##### marker_array: Combine markers #####
        marker_array = MarkerArray()
        marker_array.markers.append(trail_marker)
        marker_array.markers.append(current_marker)
        
        self.pen_trail_pub.publish(marker_array)
    

    def publish_pen_body(self):
        """Publish the pen body according to the current base position."""
        if not hasattr(self, 'current_base_pos'):
            return  # まだPenStateを受信していない場合はスキップ

        pen_body = Marker()
        pen_body.header.frame_id = "base_link"
        pen_body.header.stamp = self.get_clock().now().to_msg()
        pen_body.ns = "pen_body"
        pen_body.id = 0
        pen_body.type = Marker.CYLINDER
        pen_body.action = Marker.ADD

        # Compute the pen body position (Center between base and tip)
        pen_body.pose.position.x = (self.current_base_pos[0] + self.current_tip_pos[0]) / 2.0
        pen_body.pose.position.y = (self.current_base_pos[1] + self.current_tip_pos[1]) / 2.0
        pen_body.pose.position.z = (self.current_base_pos[2] + self.current_tip_pos[2]) / 2.0

        # Orientation: Point downwards
        pen_body.pose.orientation = self.calculate_orientation()

        # Compute the length of the pen body
        dx = self.current_tip_pos[0] - self.current_base_pos[0]
        dy = self.current_tip_pos[1] - self.current_base_pos[1]
        dz = self.current_tip_pos[2] - self.current_base_pos[2]
        pen_body_length = math.sqrt(dx*dx + dy*dy + dz*dz)

        # PEN PROPERTIES: shape, size, color
        # Set diameter. todo: load from config.yaml
        PEN_DIAMETER = 0.01  # 10mm diameter 
        
        # Set the scale of the pen body
        pen_body.scale.x = PEN_DIAMETER
        pen_body.scale.y = PEN_DIAMETER
        pen_body.scale.z = pen_body_length  # Length from base to tip

        # color (deep blue)
        pen_body.color.r = 0.0
        pen_body.color.g = 0.2
        pen_body.color.b = 0.8
        pen_body.color.a = 1.0

        # lifetime
        pen_body.lifetime.sec = 0
        pen_body.lifetime.nanosec = 0

        self.pen_body_pub.publish(pen_body)

    def calculate_orientation(self):
        """Compute quaternion orientation from base→tip direction."""
        # todo: I havent understood..... check this later
        default_axis = np.array([0.0, 0.0, 1.0])  # CYLINDERはZ軸が軸方向

        v = np.array(self.current_tip_pos) - np.array(self.current_base_pos)
        norm = np.linalg.norm(v)

        # If the vector is too small, return identity quaternion
        if norm < 1e-6:
            q = Quaternion()
            q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
            return q

        # Normalize the direction vector
        direction = v / norm

        rotation = R.align_vectors([direction], [default_axis])[0]
        quat = rotation.as_quat()  # [x, y, z, w]

        # Convert to ROS Quaternion message
        q = Quaternion()
        q.x, q.y, q.z, q.w = quat
        return q


    def publish_pen_tip(self, robot_pos):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pen_tip"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = robot_pos[0]
        marker.pose.position.y = robot_pos[1]
        marker.pose.position.z = robot_pos[2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.012
        marker.scale.y = 0.012
        marker.scale.z = 0.012

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.pen_tip_pub.publish(marker)
