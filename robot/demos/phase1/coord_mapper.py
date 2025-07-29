"""
TF2-based Coordinate Mapper for Robot Drawing
very wip!
"""
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class CoordMapper:
    def __init__(self, tf_buffer, config):
        self.tf_buffer = tf_buffer  # ROS2標準TF2
        self.config = config

    def pixel_to_robot_coords(self, pixel_x, pixel_y, pen_down=True):
        """既存APIを保持、内部でTF2活用"""
        # Step 1: pixel → canvas 3D position (custom logic)
        canvas_pos = self._pixel_to_canvas_position(pixel_x, pixel_y, pen_down)

        # Step 2: canvas → base_link (TF2 transform)
        canvas_point = PointStamped()
        canvas_point.header.frame_id = "canvas_frame"
        canvas_point.point.x, canvas_point.point.y, canvas_point.point.z = canvas_pos

        base_point = self.tf_buffer.transform(canvas_point, "base_link")
        return (base_point.point.x, base_point.point.y, base_point.point.z)

    def joints_to_pen_tip_position(self, joints):
        """TF2完全活用版"""
        # pen_tip frame定義済みなら直接取得
        pen_tip_point = PointStamped()
        pen_tip_point.header.frame_id = "pen_tip"
        pen_tip_point.point.x = pen_tip_point.point.y = pen_tip_point.point.z = 0.0

        base_point = self.tf_buffer.transform(pen_tip_point, "base_link")
        return (base_point.point.x, base_point.point.y, base_point.point.z)