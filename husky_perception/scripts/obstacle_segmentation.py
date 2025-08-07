#!/usr/bin/env python3
import rospy
import numpy as np
import threading
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import MarkerArray, Marker

class LidarClustering:
    def __init__(self):
        rospy.init_node('obstacle_segmentation')

        # Maximum distance to consider points (meters)
        self.max_distance = rospy.get_param('~max_distance', 10.0)

        # DBSCAN clustering parameters (optional clustering)
        self.cluster_eps = rospy.get_param('~cluster_eps', 0.4)
        self.min_samples = rospy.get_param('~min_samples', 10)

        # Grid size for ground filtering (meters)
        self.grid_size = rospy.get_param('~grid_size', 0.3)

        # Ground offset threshold (meters) to separate ground points from obstacles
        self.ground_offset = rospy.get_param('~ground_offset', 0.2)

        # Flag to enable visualization of clusters (optional)
        self.visualize = rospy.get_param('~visualize', True)

        # Publishers: 
        # - Markers for clusters visualization (optional)
        # - PointCloud2 for ground points
        # - PointCloud2 for obstacle points
        self.clusters_pub = rospy.Publisher('/cluster_markers', MarkerArray, queue_size=10)
        self.ground_pub = rospy.Publisher('/ground_points', PointCloud2, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/obstacle_points', PointCloud2, queue_size=10)

        # Subscriber for raw point cloud data
        self.pc_sub = rospy.Subscriber('/points', PointCloud2, self.cloud_callback)

        # Thread lock to prevent concurrent processing
        self.lock = threading.Lock()

        # Timer to limit processing frequency
        self.last_time = rospy.Time.now()

        rospy.loginfo("Optimized 'obstacle_segmentation' node started.")

    def remove_ground(self, points):
        """
        Separate ground and obstacle points using height filtering per grid cell.
        This is the core step: dividing the point cloud into ground and obstacles.
        Clustering is optional and happens later.
        """
        if len(points) == 0:
            return np.array([]), np.array([])

        # Create 2D grid bins based on XY coordinates
        x_bins = np.arange(np.min(points[:, 0]), np.max(points[:, 0]) + self.grid_size, self.grid_size)
        y_bins = np.arange(np.min(points[:, 1]), np.max(points[:, 1]) + self.grid_size, self.grid_size)

        # Assign each point to a grid cell
        x_idx = np.digitize(points[:, 0], x_bins) - 1
        y_idx = np.digitize(points[:, 1], y_bins) - 1
        cell_ids = x_idx + y_idx * len(x_bins)

        unique_ids = np.unique(cell_ids)
        obstacles = []
        ground = []

        # For each cell, separate points below ground threshold and above as obstacles
        for cid in unique_ids:
            mask = cell_ids == cid
            cell_points = points[mask]
            if len(cell_points) == 0:
                continue
            min_z = np.min(cell_points[:, 2])
            ground_threshold = min_z + self.ground_offset
            ground_mask = cell_points[:, 2] < ground_threshold
            ground.append(cell_points[ground_mask])
            obstacles.append(cell_points[~ground_mask])

        # Concatenate all obstacle and ground points
        return np.vstack(obstacles) if obstacles else np.array([]), \
               np.vstack(ground) if ground else np.array([])

    def cloud_callback(self, msg):
        """
        Callback for incoming point cloud messages.
        Limits processing frequency to once every 0.5 seconds.
        Launches processing in a separate thread.
        """
        now = rospy.Time.now()
        if (now - self.last_time).to_sec() < 0.5:
            return
        self.last_time = now

        threading.Thread(target=self.process_cloud, args=(msg,)).start()

    def process_cloud(self, msg):
        """
        Processes the incoming point cloud:
        - Filters points by max distance
        - Separates ground and obstacles (essential step)
        - Optionally performs clustering on obstacles
        - Publishes filtered clouds and visualization markers
        """
        if not self.lock.acquire(blocking=False):
            return  # Avoid simultaneous executions

        try:
            # Convert ROS PointCloud2 to numpy array
            points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            if len(points) == 0:
                return

            # Filter points by horizontal distance
            mask = np.linalg.norm(points[:, :2], axis=1) < self.max_distance
            filtered = points[mask]

            # Separate ground and obstacle points
            obstacles, ground = self.remove_ground(filtered)

            # Publish separated ground and obstacle point clouds
            self.publish_pointcloud(msg.header, ground, self.ground_pub)
            self.publish_pointcloud(msg.header, obstacles, self.obstacle_pub)

            # Optional clustering of obstacle points for visualization or further processing
            if len(obstacles) >= self.min_samples:
                # Subsample large point clouds for performance
                if len(obstacles) > 2000:
                    indices = np.random.choice(len(obstacles), 2000, replace=False)
                    obstacles = obstacles[indices]
                    rospy.logdebug("Subsampling applied to obstacles before clustering.")

                # Perform DBSCAN clustering on obstacles
                db = DBSCAN(eps=self.cluster_eps, min_samples=self.min_samples).fit(obstacles)
                labels = db.labels_

                # Publish cluster markers if visualization is enabled
                if self.visualize:
                    self.publish_markers(msg.header, obstacles, labels)

        except Exception as e:
            rospy.logerr(f"Processing error: {str(e)}")
        finally:
            self.lock.release()

    def publish_markers(self, header, points, labels):
        """
        Publish visualization markers for each cluster detected.
        This step is optional and helps visualize the clusters in RViz.
        """
        markers = MarkerArray()
        unique_labels = set(labels)

        for label in unique_labels:
            if label == -1:
                continue  # Ignore noise

            cluster = points[labels == label]
            centroid = np.mean(cluster, axis=0)
            max_pt = np.max(cluster, axis=0)
            min_pt = np.min(cluster, axis=0)

            marker = Marker()
            marker.header = header
            marker.ns = "clusters"
            marker.id = label
            marker.type = Marker.CUBE
            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = centroid[2]
            marker.scale.x = max_pt[0] - min_pt[0]
            marker.scale.y = max_pt[1] - min_pt[1]
            marker.scale.z = max_pt[2] - min_pt[2]

            # Assign color based on label to distinguish clusters
            marker.color.r = (label % 3) / 3.0
            marker.color.g = ((label + 1) % 3) / 3.0
            marker.color.b = ((label + 2) % 3) / 3.0
            marker.color.a = 0.4

            marker.lifetime = rospy.Duration(0.5)
            markers.markers.append(marker)

        self.clusters_pub.publish(markers)
        rospy.logdebug(f"Published {len(markers.markers)} clusters.")

    def publish_pointcloud(self, header, points, publisher):
        """
        Helper function to publish a PointCloud2 message from numpy points.
        """
        if len(points) == 0:
            return
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        pc2 = point_cloud2.create_cloud(header, fields, points)
        publisher.publish(pc2)

if __name__ == '__main__':
    try:
        node = LidarClustering()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
