import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import numpy as np
import cv2
import math
import threading
import time
from functools import partial

class ThreePointMerger(Node):
    def __init__(self):
        super().__init__('three_point_merger')

        # --- CONFIGURATION (Start Positions) ---
        self.ROBOTS = [
            {'name': 'burger', 'x': -2.0, 'y': -9.0, 'yaw': 0.0},
            {'name': 'waffle_pi', 'x': 3.0, 'y': 9.0, 'yaw': 0.0},
        ]

        # Static Broadcaster (Keeps RViz Happy)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tfs()

        self.latest_maps = {}
        self.subs = []
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', 10)
        
        for r in self.ROBOTS:
            topic = f"/{r['name']}/map"
            cb = partial(self.map_callback, robot_name=r['name'])
            self.subs.append(self.create_subscription(OccupancyGrid, topic, cb, 10))

        self.create_timer(1.0, self.merge_maps)

    def publish_static_tfs(self):
        transforms = []
        now = self.get_clock().now().to_msg()
        for r in self.ROBOTS:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'world'
            t.child_frame_id = f"{r['name']}/map"
            t.transform.translation.x = float(r['x'])
            t.transform.translation.y = float(r['y'])
            q = tf_transformations.quaternion_from_euler(0, 0, r['yaw'])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            transforms.append(t)
        self.static_broadcaster.sendTransform(transforms)

    def map_callback(self, msg, robot_name):
        self.latest_maps[robot_name] = msg

    def get_world_point(self, local_x, local_y, r_conf, map_info):
        """
        Converts a point from Map Pixel (local_x, local_y) to World (m).
        1. Scale to meters.
        2. Shift by Map Origin.
        3. Rotate by Robot Yaw.
        4. Shift by Robot Start.
        """
        res = map_info.resolution
        ox = map_info.origin.position.x
        oy = map_info.origin.position.y
        
        # 1. To Map Meters (relative to map frame)
        mx = (local_x * res) + ox
        my = (local_y * res) + oy
        
        # 2. Rotate (Robot Yaw)
        yaw = r_conf['yaw']
        cos_a = math.cos(yaw)
        sin_a = math.sin(yaw)
        
        # Rotation Logic: x' = x*cos - y*sin
        rx = mx * cos_a - my * sin_a
        ry = mx * sin_a + my * cos_a
        
        # 3. Translate (Robot Start)
        wx = rx + r_conf['x']
        wy = ry + r_conf['y']
        
        return [wx, wy]

    def merge_maps(self):
        if not self.latest_maps: return

        valid_maps = []
        resolution = 0.05
        
        # --- STEP 1: CALCULATE CORNERS FOR ALL MAPS ---
        all_world_points = []
        
        for r in self.ROBOTS:
            name = r['name']
            if name not in self.latest_maps: continue
            msg = self.latest_maps[name]
            h, w = msg.info.height, msg.info.width
            resolution = msg.info.resolution

            # Define 3 Key Points in Pixel Space (Src)
            # P1: Bottom-Left (0,0)
            # P2: Bottom-Right (w, 0)
            # P3: Top-Left (0, h)
            src_tri = np.float32([[0, 0], [w, 0], [0, h]])
            
            # Calculate where these are in World Space
            p1_w = self.get_world_point(0, 0, r, msg.info)
            p2_w = self.get_world_point(w, 0, r, msg.info)
            p3_w = self.get_world_point(0, h, r, msg.info)
            
            # Save for later
            valid_maps.append({
                'msg': msg,
                'src_tri': src_tri,
                'dst_world': np.float32([p1_w, p2_w, p3_w])
            })
            
            # Collect for bounding box
            # Also calculate Top-Right to ensure full coverage box
            p4_w = self.get_world_point(w, h, r, msg.info)
            all_world_points.extend([p1_w, p2_w, p3_w, p4_w])

        if not valid_maps: return

        # --- STEP 2: DEFINE CANVAS ---
        pts = np.array(all_world_points)
        min_wx = np.min(pts[:, 0])
        min_wy = np.min(pts[:, 1])
        max_wx = np.max(pts[:, 0])
        max_wy = np.max(pts[:, 1])

        canvas_w = int(np.ceil((max_wx - min_wx) / resolution))
        canvas_h = int(np.ceil((max_wy - min_wy) / resolution))
        
        # Init Canvas (Unknown = 127)
        canvas = np.full((canvas_h, canvas_w), 127, dtype=np.uint8)

        # --- STEP 3: WARP MAPS ---
        for item in valid_maps:
            msg = item['msg']
            src_tri = item['src_tri']     # The 3 pixel corners of source
            world_tri = item['dst_world'] # The 3 world coordinates those corners belong to
            
            # Convert World Coordinates to Canvas Pixel Coordinates
            # CanvasX = (WorldX - MinX) / Res
            dst_tri = []
            for wp in world_tri:
                cx = (wp[0] - min_wx) / resolution
                cy = (wp[1] - min_wy) / resolution
                dst_tri.append([cx, cy])
            dst_tri = np.float32(dst_tri)

            # GET AFFINE TRANSFORM (Magic OpenCV Function)
            M = cv2.getAffineTransform(src_tri, dst_tri)

            # Prepare Source Image
            h, w = msg.info.height, msg.info.width
            arr = np.array(msg.data, dtype=np.int8).reshape(h, w)
            img = np.full((h, w), 127, dtype=np.uint8)
            img[arr == 0] = 255   # Free
            img[arr == 100] = 0   # Occupied

            # Warp
            warped = cv2.warpAffine(img, M, (canvas_w, canvas_h), 
                                    flags=cv2.INTER_NEAREST, borderValue=127)

            # Paste Logic
            canvas[warped == 0] = 0 # Occupied overwrites everything
            mask_free = (warped == 255) & (canvas == 127)
            canvas[mask_free] = 255

        # --- STEP 4: PUBLISH ---
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.info.resolution = resolution
        out.info.width = canvas_w
        out.info.height = canvas_h
        out.info.origin.position.x = float(min_wx)
        out.info.origin.position.y = float(min_wy)
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0

        flat = np.full(canvas.size, -1, dtype=np.int8)
        flat[canvas.flatten() == 0] = 100
        flat[canvas.flatten() == 255] = 0
        out.data = flat.tolist()
        self.pub_map.publish(out)

        # Debug logs to verify positions
        self.get_logger().info(f"Canvas: ({min_wx:.1f}, {min_wy:.1f}) to ({max_wx:.1f}, {max_wy:.1f})", throttle_duration_sec=2)

def main(args=None):
    rclpy.init(args=args)
    node = ThreePointMerger()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()