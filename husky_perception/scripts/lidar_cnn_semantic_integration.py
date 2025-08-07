#!/usr/bin/env python3

# ROS imports
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2, LaserScan, Image
from sensor_msgs import point_cloud2
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import message_filters
import math
from dynamic_reconfigure.server import Server
from husky_perception.cfg import SemanticFusionConfig

class SemanticFusionNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("semantic_fusion_node")

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Load ROS parameters
        self.lidar_frame = rospy.get_param("~lidar_frame", "velodyne")
        self.scan_angle_min = -math.pi
        self.scan_angle_max = math.pi
        self.scan_angle_increment = math.radians(1.0)
        self.scan_range_max = 30.0
        self.raw_downsample = rospy.get_param("~raw_downsample", 3)  # Publish 1 in every N raw scan rays
        self.spread_factor = rospy.get_param("~spread_factor", 1.5)  # Factor to spread semantic rays
        self.min_score_threshold = rospy.get_param("~min_score_threshold", 0.4)  # Minimum confidence for detections

        # COCO class priority settings
        self.class_priority = {
            'high': [0, 1, 2, 3, 5, 7, 16, 17, 18, 19],
            'medium': [6, 9, 10, 11, 12, 56, 58, 59],
            'low': list(set(range(80)) - set([0,1,2,3,5,7,16,17,18,19,6,9,10,11,12,56,58,59]))
        }

        # Number of semantic rays per priority
        self.priority_rays = {
            'high': 15,
            'medium': 9,
            'low': 5
        }

        # Width of virtual coverage (in meters) per priority
        self.priority_coverage_width = {
            'high': 2.0,
            'medium': 1.2,
            'low': 0.5
        }

        self.latest_obstacles = None

        # Subscribe to point cloud of detected obstacles
        rospy.Subscriber("/obstacle_points", PointCloud2, self.obstacle_points_callback)

        # Sync RGB image, point cloud, and 2D detections
        image_sub = message_filters.Subscriber("/realsense/color/image_raw", Image)
        depth_pc_sub = message_filters.Subscriber("/realsense/depth/color/points", PointCloud2)
        detections_sub = message_filters.Subscriber("/detections", Detection2DArray)

        sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_pc_sub, detections_sub], queue_size=5, slop=0.8
        )
        sync.registerCallback(self.semantic_callback)

        # Publishers for semantic and raw laser scans
        self.semantic_scan_pub = rospy.Publisher("/semantic_scan", LaserScan, queue_size=1)
        self.raw_scan_pub = rospy.Publisher("/raw_scan", LaserScan, queue_size=1)
        self.marker_pub = rospy.Publisher("/semantic_markers", MarkerArray, queue_size=1)

        # Timer to periodically publish raw scan
        self.raw_scan_timer = rospy.Timer(rospy.Duration(0.05), self.publish_raw_scan)

        rospy.loginfo("🟢 Nodo semantic_fusion_node iniciado.")
        # Dynamic reconfigure server
        self.dyn_server = Server(SemanticFusionConfig, self.reconfig_callback)

    def reconfig_callback(self, config, level):
        self.min_score_threshold = config.min_score_threshold
        self.raw_downsample = config.raw_downsample
        self.spread_factor = config.spread_factor

        self.priority_rays['high'] = config.priority_rays_high
        self.priority_rays['medium'] = config.priority_rays_medium
        self.priority_rays['low'] = config.priority_rays_low

        self.priority_coverage['high'] = config.priority_coverage_high
        self.priority_coverage['medium'] = config.priority_coverage_medium
        self.priority_coverage['low'] = config.priority_coverage_low

        rospy.loginfo("[SemanticFusion] Reconfigured parameters received.")
        return config


    def obstacle_points_callback(self, cloud_msg):
        # Store the most recent obstacle cloud for raw scan generation
        self.latest_obstacles = cloud_msg

    def publish_raw_scan(self, event):
        # Publish synthetic raw scan from obstacle cloud
        if self.latest_obstacles is None:
            return

        ranges = [float('inf')] * int((self.scan_angle_max - self.scan_angle_min) / self.scan_angle_increment)

        for point in point_cloud2.read_points(self.latest_obstacles, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            angle = math.atan2(y, x)
            dist = math.hypot(x, y)
            if dist < 0.2 or dist > self.scan_range_max:
                continue
            index = int((angle - self.scan_angle_min) / self.scan_angle_increment)

            if self.raw_downsample > 1 and index % self.raw_downsample != 0:
                continue

            if 0 <= index < len(ranges):
                if dist < ranges[index]:
                    ranges[index] = dist

        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.lidar_frame
        scan.angle_min = self.scan_angle_min
        scan.angle_max = self.scan_angle_max
        scan.angle_increment = self.scan_angle_increment
        scan.range_min = 0.2
        scan.range_max = self.scan_range_max
        scan.ranges = ranges

        self.raw_scan_pub.publish(scan)

    def semantic_callback(self, image_msg, depth_pc_msg, detections_msg):
        # Fuse 2D detections with 3D depth points to publish semantic scan
        try:
            height = image_msg.height
            width = image_msg.width
            pc_array = np.array(list(point_cloud2.read_points(
                depth_pc_msg, field_names=("x", "y", "z"), skip_nans=False)))
            pc_array = pc_array.reshape((height, width, 3))
        except Exception as e:
            rospy.logerr(f"❌ Error processing organized cloud: {e}")
            return

        marker_array = MarkerArray()
        semantic_ranges = [float('inf')] * int((self.scan_angle_max - self.scan_angle_min) / self.scan_angle_increment)
        marker_id = 0

        max_spread_angle = math.radians(30)
        min_spread_angle = math.radians(3)

        for detection in detections_msg.detections:
            cx = int(detection.bbox.center.x)
            cy = int(detection.bbox.center.y)
            if cx < 0 or cx >= width or cy < 0 or cy >= height:
                continue

            # Get detection score and class ID
            class_id = detection.results[0].id if detection.results else -1
            score = detection.results[0].score if detection.results else 0.0
            if score < self.min_score_threshold:
                continue

            x, y, z = pc_array[cy, cx]
            if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                continue

            pt_cam = PointStamped()
            pt_cam.header = image_msg.header
            pt_cam.point.x = x
            pt_cam.point.y = y
            pt_cam.point.z = z

            try:
                # Transform from camera to LiDAR frame
                trans = self.tf_buffer.lookup_transform(
                    self.lidar_frame, pt_cam.header.frame_id, pt_cam.header.stamp, rospy.Duration(0.2))
                pt_lidar = tf2_geometry_msgs.do_transform_point(pt_cam, trans)
                lx = pt_lidar.point.x
                ly = pt_lidar.point.y
                lz = pt_lidar.point.z
            except Exception as e:
                rospy.logwarn(f"⚠️ TF transform failed: {e}")
                continue

            angle = math.atan2(ly, lx)
            dist = math.hypot(lx, ly)

            # Assign class priority
            priority = 'low'
            for key, class_list in self.class_priority.items():
                if class_id in class_list:
                    priority = key
                    break

            n_points = self.priority_rays[priority]
            coverage_width = self.priority_coverage_width[priority]

            # Compute angular spread for coverage area
            if dist > 0.1:
                spread_angle = 2 * math.atan((coverage_width / 2.0) / dist)
            else:
                spread_angle = min_spread_angle

            spread_angle = max(min(spread_angle, max_spread_angle), min_spread_angle)
            start_angle = angle - spread_angle / 2
            angle_step = spread_angle / (n_points - 1) if n_points > 1 else 0

            for i in range(n_points):
                ray_angle = start_angle + i * angle_step
                lx_off = dist * math.cos(ray_angle)
                ly_off = dist * math.sin(ray_angle)
                dist_off = math.hypot(lx_off, ly_off)
                index = int((ray_angle - self.scan_angle_min) / self.scan_angle_increment)
                if 0 <= index < len(semantic_ranges):
                    if dist_off < semantic_ranges[index]:
                        semantic_ranges[index] = dist_off

            # Add a colored sphere marker at detection
            sphere = Marker()
            sphere.header.frame_id = self.lidar_frame
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "semantic_spheres"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = lx
            sphere.pose.position.y = ly
            sphere.pose.position.z = lz
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25

            if priority == 'high':
                sphere.color.r = 1.0
                sphere.color.g = 0.0
                sphere.color.b = 0.0
            elif priority == 'medium':
                sphere.color.r = 1.0
                sphere.color.g = 1.0
                sphere.color.b = 0.0
            else:
                sphere.color.r = 0.0
                sphere.color.g = 0.7
                sphere.color.b = 1.0

            sphere.color.a = 0.9
            sphere.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(sphere)

            # Add class ID text marker above the sphere
            text_marker = Marker()
            text_marker.header.frame_id = self.lidar_frame
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "semantic_texts"
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = lx
            text_marker.pose.position.y = ly
            text_marker.pose.position.z = lz + 0.3
            text_marker.scale.z = 0.25
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"id={class_id}, d={dist:.1f}m"
            text_marker.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

        # Publish semantic scan
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.lidar_frame
        scan.angle_min = self.scan_angle_min
        scan.angle_max = self.scan_angle_max
        scan.angle_increment = self.scan_angle_increment
        scan.range_min = 0.2
        scan.range_max = self.scan_range_max
        scan.ranges = semantic_ranges
        self.semantic_scan_pub.publish(scan)

if __name__ == "__main__":
    try:
        SemanticFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
