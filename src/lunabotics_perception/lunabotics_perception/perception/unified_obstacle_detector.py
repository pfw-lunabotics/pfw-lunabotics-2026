#!/usr/bin/env python3
"""
Unified Obstacle Detector for Lunabotics 2026
==================================================

Core insight: Craters are inverted rocks. Treat them the same.

Pipeline:
1. ROI Crop - 360-degree radial crop around robot
2. Ground Segmentation - RANSAC plane fit
3. Rock Detection - Points ABOVE ground
4. Crater Inversion - Points BELOW ground → mirror across ground plane
5. Clustering - DBSCAN to group nearby points, filter noise
6. Unified Cloud - Both rocks and inverted craters
7. Publish - Single obstacle cloud to Nav2 + HazardSummary to BT

Author: PFW Lunabotics Team
Date: February 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from luna_msgs.msg import HazardSummary
import numpy as np
from sklearn.linear_model import RANSACRegressor
from sklearn.cluster import DBSCAN


class UnifiedObstacleDetector(Node):
    """
    Unified obstacle detection using crater inversion technique.

    Subscribes to LiDAR point cloud, detects rocks and craters,
    treats them identically as obstacles to avoid.
    """

    def __init__(self):
        super().__init__('unified_obstacle_detector')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # ROI Parameters — 360-degree radial
                ('roi.range_min', 0.05),     # Min radial distance (m) - L2 min range
                ('roi.range_max', 4.0),      # Max radial distance (m)
                ('roi.height_min', -0.5),    # Min height (for craters)
                ('roi.height_max', 1.0),     # Max height (for rocks)

                # Ground Segmentation Parameters
                ('ground.ransac_threshold', 0.08),
                ('ground.ransac_iterations', 300),
                ('ground.min_points', 50),
                ('ground.height_tolerance', 0.12),

                # Crater Detection Parameters
                ('crater.depth_threshold', 0.10),
                ('crater.inversion_enabled', True),

                # Rock Detection Parameters
                ('rock.height_threshold', 0.10),

                # Clustering Parameters
                ('cluster.eps', 0.15),           # Max distance between points in a cluster (m)
                ('cluster.min_points', 5),       # Min points to form a valid cluster

                # Blocked corridor Parameters
                ('blocked.lookahead', 2.0),      # Forward distance to check (m)
                ('blocked.half_width', 0.4),     # Half robot width + margin (m)

                # Inflation Parameters
                ('inflation.radius', 0.30),
                ('inflation.enabled', False),

                # Publishing Parameters
                ('publish_rate', 10.0),
                ('debug_mode', False),

                # Legacy ROI params (ignored, kept for YAML backwards compat)
                ('roi.forward_min', 0.05),
                ('roi.forward_max', 4.0),
                ('roi.lateral_max', 2.5),
            ]
        )

        # Get parameters
        self.roi_range_min = self.get_parameter('roi.range_min').value
        self.roi_range_max = self.get_parameter('roi.range_max').value
        self.roi_height_min = self.get_parameter('roi.height_min').value
        self.roi_height_max = self.get_parameter('roi.height_max').value

        self.ground_ransac_threshold = self.get_parameter('ground.ransac_threshold').value
        self.ground_ransac_iterations = self.get_parameter('ground.ransac_iterations').value
        self.ground_min_points = self.get_parameter('ground.min_points').value
        self.ground_height_tolerance = self.get_parameter('ground.height_tolerance').value

        self.crater_depth_threshold = self.get_parameter('crater.depth_threshold').value
        self.crater_inversion_enabled = self.get_parameter('crater.inversion_enabled').value

        self.rock_height_threshold = self.get_parameter('rock.height_threshold').value

        self.cluster_eps = self.get_parameter('cluster.eps').value
        self.cluster_min_points = self.get_parameter('cluster.min_points').value

        self.blocked_lookahead = self.get_parameter('blocked.lookahead').value
        self.blocked_half_width = self.get_parameter('blocked.half_width').value

        self.inflation_radius = self.get_parameter('inflation.radius').value
        self.inflation_enabled = self.get_parameter('inflation.enabled').value

        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.lidar_callback,
            10
        )

        # Publishers
        self.unified_obstacles_pub = self.create_publisher(
            PointCloud2, '/perception/unified_obstacles', 10
        )
        self.hazard_summary_pub = self.create_publisher(
            HazardSummary, '/perception/hazard_summary', 10
        )

        # Debug publishers (if enabled)
        if self.debug_mode:
            self.ground_pub = self.create_publisher(PointCloud2, '/perception/debug/ground', 10)
            self.rocks_pub = self.create_publisher(PointCloud2, '/perception/debug/rocks', 10)
            self.craters_pub = self.create_publisher(PointCloud2, '/perception/debug/craters', 10)
            self.roi_pub = self.create_publisher(PointCloud2, '/perception/debug/roi', 10)

        # Frame skipping for CPU budget — process every Nth cloud
        self._skip_count = 0
        self._process_every_n = max(1, int(12.0 / self.publish_rate))  # ~12Hz input / target Hz
        self.get_logger().info(f'Frame skip: process every {self._process_every_n} frames')

        # Statistics
        self.frame_count = 0
        self.last_stats_time = self.get_clock().now()

        self.get_logger().info('Unified Obstacle Detector initialized')
        self.get_logger().info(
            f'ROI: 360-degree, range [{self.roi_range_min}, {self.roi_range_max}]m, '
            f'height [{self.roi_height_min}, {self.roi_height_max}]m'
        )
        self.get_logger().info(
            f'Clustering: eps={self.cluster_eps}m, min_points={self.cluster_min_points}'
        )
        self.get_logger().info(f'Ground tolerance: {self.ground_height_tolerance}m')
        self.get_logger().info(
            f'Crater inversion: {"ENABLED" if self.crater_inversion_enabled else "DISABLED"}'
        )

    def lidar_callback(self, msg):
        """Process incoming LiDAR point cloud."""
        # Frame skip for CPU budget
        self._skip_count += 1
        if self._skip_count % self._process_every_n != 0:
            return

        try:
            # Step 1: Convert ROS PointCloud2 to numpy array
            points = self.pointcloud2_to_numpy(msg)

            if points.shape[0] == 0:
                self.get_logger().warn('Received empty point cloud')
                return

            # Step 2: 360-degree ROI Crop
            roi_points = self.crop_roi(points)

            if roi_points.shape[0] < self.ground_min_points:
                self.get_logger().warn(f'Too few points after ROI crop: {roi_points.shape[0]}')
                return

            # Step 3: Ground Segmentation
            ground_mask, ground_plane, inlier_ratio = self.segment_ground(roi_points)

            if ground_plane is None:
                self.get_logger().warn('Failed to detect ground plane')
                return

            # Step 4: Classify points
            above_ground, below_ground = self.classify_points(
                roi_points, ground_mask, ground_plane
            )

            # Step 5: Detect rocks (above ground) + cluster
            rocks = self.detect_rocks(above_ground)

            # Step 6: Detect and invert craters (below ground) + cluster
            craters_inverted = self.detect_and_invert_craters(below_ground, ground_plane)

            # Step 7: Cluster and filter noise
            # Skip DBSCAN on Jetson to save CPU — Nav2 inflation handles safety margin
            if self.cluster_eps > 0 and rocks.shape[0] + craters_inverted.shape[0] < 200:
                rocks_clustered = self.cluster_filter(rocks)
                craters_clustered = self.cluster_filter(craters_inverted)
            else:
                rocks_clustered = rocks
                craters_clustered = craters_inverted

            # Step 8: Unify obstacles
            unified_obstacles = self.unify_obstacles(rocks_clustered, craters_clustered)

            # Step 9: Inflate obstacles
            if self.inflation_enabled and unified_obstacles.shape[0] > 0:
                unified_obstacles = self.inflate_obstacles(unified_obstacles)

            # Use current wall-clock time for obstacle output
            out_header = msg.header
            out_header.stamp = self.get_clock().now().to_msg()

            # Step 10: Flatten obstacle z to 0 for 2D costmap
            # The costmap only uses x,y — keeping LiDAR-frame z values causes
            # min_obstacle_height filtering to drop all points (they're negative
            # in odom frame because the LiDAR is mounted above ground)
            if unified_obstacles.shape[0] > 0:
                unified_obstacles[:, 2] = 0.0

            # Step 11: Publish unified obstacle cloud
            self.publish_obstacle_cloud(unified_obstacles, out_header)

            # Step 12: Publish hazard summary for BT
            self.publish_hazard_summary(
                unified_obstacles, rocks_clustered, craters_clustered,
                inlier_ratio, out_header
            )

            # Debug publishing
            if self.debug_mode:
                self.publish_debug_clouds(
                    roi_points, ground_mask, rocks_clustered,
                    craters_clustered, out_header
                )

            # Statistics
            self.frame_count += 1
            now = self.get_clock().now()
            if (now - self.last_stats_time).nanoseconds / 1e9 >= 5.0:
                self.print_statistics(
                    roi_points.shape[0], rocks_clustered.shape[0],
                    craters_clustered.shape[0], unified_obstacles.shape[0]
                )
                self.last_stats_time = now

        except Exception as e:
            self.get_logger().error(f'Error in lidar_callback: {str(e)}')

    def pointcloud2_to_numpy(self, cloud_msg):
        """Convert ROS PointCloud2 to numpy array (N x 3: x, y, z).

        Uses structured numpy array instead of per-point Python iteration
        for ~10-50x speedup on large clouds.

        Validates that required x, y, z fields exist before extraction.
        Handles both compact (sim) and strided (real L2 with intensity/ring) layouts.
        """
        n_points = cloud_msg.width * cloud_msg.height
        if n_points == 0:
            return np.empty((0, 3), dtype=np.float32)

        # Build structured dtype from PointCloud2 fields
        field_map = {f.name: f.offset for f in cloud_msg.fields}
        point_step = cloud_msg.point_step

        # Validate required fields exist
        for axis in ('x', 'y', 'z'):
            if axis not in field_map:
                self.get_logger().error(
                    f'PointCloud2 missing required field "{axis}". '
                    f'Available fields: {list(field_map.keys())}'
                )
                return np.empty((0, 3), dtype=np.float32)

        # Fast path: xyz are the first 3 float32 fields (compact layout, e.g. Gazebo)
        if (field_map.get('x') == 0 and field_map.get('y') == 4
                and field_map.get('z') == 8 and point_step == 12):
            xyz = np.frombuffer(cloud_msg.data, dtype=np.float32).reshape(-1, 3).copy()
        else:
            # General path: extract x, y, z from strided buffer
            # Handles real L2 layout (x, y, z, intensity, ring, etc.)
            raw = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(n_points, point_step)
            xyz = np.empty((n_points, 3), dtype=np.float32)
            for i, axis in enumerate(('x', 'y', 'z')):
                offset = field_map[axis]
                xyz[:, i] = raw[:, offset:offset + 4].view(np.float32).flatten()

        # Filter NaN and Inf points (real hardware can produce both)
        valid_mask = np.isfinite(xyz).all(axis=1)
        return xyz[valid_mask]

    def crop_roi(self, points):
        """
        Crop points to 360-degree Region of Interest (ROI).

        Uses radial distance in XY plane (not forward-only).
        The L2 LiDAR has 360 x 96 deg FOV — use all of it.
        Height filter still applies to focus on ground-level features.
        """
        # Radial distance in XY plane
        radial_dist = np.sqrt(points[:, 0]**2 + points[:, 1]**2)

        range_mask = (radial_dist >= self.roi_range_min) & (radial_dist <= self.roi_range_max)
        height_mask = (points[:, 2] >= self.roi_height_min) & (points[:, 2] <= self.roi_height_max)

        return points[range_mask & height_mask]

    def segment_ground(self, points):
        """
        Segment ground plane using RANSAC.

        Returns:
            ground_mask:  Boolean mask of ground inlier points
            ground_plane: (a, b, c, d) coefficients for plane ax + by + cz + d = 0
            inlier_ratio: fraction of ROI points that are ground (confidence proxy)
        """
        if points.shape[0] < self.ground_min_points:
            return None, None, 0.0

        # Fit plane using RANSAC: z = ax + by + c  →  ax + by - z + c = 0
        X = points[:, :2]   # (x, y) as features
        z = points[:, 2]    # z as target

        try:
            ransac = RANSACRegressor(
                max_trials=self.ground_ransac_iterations,
                residual_threshold=self.ground_ransac_threshold,
                random_state=42
            )
            ransac.fit(X, z)

            inlier_mask = ransac.inlier_mask_
            inlier_ratio = float(np.sum(inlier_mask)) / points.shape[0]

            # Plane equation: z = ax + by + d  →  ax + by + (-1)z + d = 0
            a = ransac.estimator_.coef_[0]
            b = ransac.estimator_.coef_[1]
            c = -1.0
            d = ransac.estimator_.intercept_

            return inlier_mask, (a, b, c, d), inlier_ratio

        except Exception as e:
            self.get_logger().error(f'RANSAC failed: {str(e)}')
            return None, None, 0.0

    def classify_points(self, points, ground_mask, ground_plane):
        """
        Classify all ROI points as above or below the ground plane.

        Uses signed distance:
          positive → above ground (rocks)
          negative → below ground (craters)

        Returns:
            above_ground: points with distance > +height_tolerance
            below_ground: points with distance < -height_tolerance
        """
        a, b, c, d = ground_plane
        norm = np.sqrt(a**2 + b**2 + c**2)

        # Signed distance for every point
        signed_dist = (
            a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d
        ) / norm

        above_ground = points[signed_dist > self.ground_height_tolerance]
        below_ground = points[signed_dist < -self.ground_height_tolerance]

        return above_ground, below_ground

    def detect_rocks(self, above_ground):
        """Detect rocks from points above ground."""
        if above_ground.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)
        return above_ground

    def detect_and_invert_craters(self, below_ground, ground_plane):
        """
        Detect craters and mirror them above the ground plane.

        For each crater point P below the ground:
          1. Compute its signed distance below the plane (depth).
          2. Discard if depth < crater_depth_threshold (too shallow).
          3. Mirror P to the same depth ABOVE the plane.
        """
        if below_ground.shape[0] == 0 or not self.crater_inversion_enabled:
            return np.empty((0, 3), dtype=np.float32)

        a, b, c, d = ground_plane
        norm = np.sqrt(a**2 + b**2 + c**2)

        # Signed distance (negative for below-ground points)
        signed_dist = (
            a * below_ground[:, 0] + b * below_ground[:, 1] +
            c * below_ground[:, 2] + d
        ) / norm

        # Keep only craters deep enough (depth = |signed_dist|)
        depth_mask = signed_dist < -self.crater_depth_threshold
        deep_points = below_ground[depth_mask]
        deep_dist = signed_dist[depth_mask]

        if deep_points.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)

        # Ground Z at each crater point's (x, y) position
        ground_z_at_xy = a * deep_points[:, 0] + b * deep_points[:, 1] + d

        # Mirror: z_inverted = ground_z + |depth|
        inverted = deep_points.copy()
        inverted[:, 2] = ground_z_at_xy - deep_dist

        return inverted

    def cluster_filter(self, points):
        """
        Cluster points using DBSCAN and filter out small clusters (noise).

        Returns only points belonging to clusters with >= min_points members.
        This eliminates single stray points and small noise spikes that
        would otherwise be treated as obstacles.
        """
        if points.shape[0] < self.cluster_min_points:
            return np.empty((0, 3), dtype=np.float32)

        # DBSCAN on XY only — height differences within a rock don't split it
        labels = DBSCAN(
            eps=self.cluster_eps,
            min_samples=self.cluster_min_points,
        ).fit_predict(points[:, :2])

        # Keep points in valid clusters (label >= 0), discard noise (label == -1)
        valid_mask = labels >= 0
        return points[valid_mask]

    def unify_obstacles(self, rocks, craters_inverted):
        """Combine rocks and inverted craters into a single obstacle cloud."""
        parts = [p for p in (rocks, craters_inverted) if p.shape[0] > 0]
        if not parts:
            return np.empty((0, 3), dtype=np.float32)
        return np.vstack(parts)

    def inflate_obstacles(self, obstacles):
        """Inflate obstacles by adding a safety margin (pass-through to Nav2)."""
        return obstacles

    def publish_obstacle_cloud(self, obstacles, header):
        """Publish unified obstacle cloud for Nav2."""
        if obstacles.shape[0] == 0:
            empty_msg = PointCloud2()
            empty_msg.header = header
            empty_msg.header.frame_id = 'base_link'
            self.unified_obstacles_pub.publish(empty_msg)
            return

        msg = self.numpy_to_pointcloud2(obstacles, header.stamp, 'base_link')
        self.unified_obstacles_pub.publish(msg)

    def publish_hazard_summary(self, unified, rocks, craters, inlier_ratio, header):
        """Compute and publish HazardSummary for BT consumption."""
        msg = HazardSummary()
        msg.header = header
        msg.header.frame_id = 'base_link'

        msg.rock_count = int(rocks.shape[0])
        msg.crater_count = int(craters.shape[0])
        msg.obstacle_count = int(unified.shape[0])
        msg.confidence = float(inlier_ratio)

        if unified.shape[0] == 0:
            msg.min_distance_to_obstacle = -1.0
            msg.path_blocked_ahead = False
        else:
            # XY distance from robot origin to each obstacle point
            xy_dist = np.sqrt(unified[:, 0]**2 + unified[:, 1]**2)
            msg.min_distance_to_obstacle = float(np.min(xy_dist))

            # Path blocked: obstacle inside forward corridor
            forward_mask = (
                (unified[:, 0] >= self.roi_range_min) &
                (unified[:, 0] <= self.blocked_lookahead) &
                (np.abs(unified[:, 1]) <= self.blocked_half_width)
            )
            msg.path_blocked_ahead = bool(np.any(forward_mask))

        self.hazard_summary_pub.publish(msg)

    def publish_debug_clouds(self, roi_points, ground_mask, rocks, craters_inverted, header):
        """Publish debug point clouds for RViz visualization."""
        if roi_points.shape[0] > 0:
            self.roi_pub.publish(
                self.numpy_to_pointcloud2(roi_points, header.stamp, 'base_link')
            )

        if ground_mask is not None and np.sum(ground_mask) > 0:
            ground_points = roi_points[ground_mask]
            self.ground_pub.publish(
                self.numpy_to_pointcloud2(ground_points, header.stamp, 'base_link')
            )

        if rocks.shape[0] > 0:
            self.rocks_pub.publish(
                self.numpy_to_pointcloud2(rocks, header.stamp, 'base_link')
            )

        if craters_inverted.shape[0] > 0:
            self.craters_pub.publish(
                self.numpy_to_pointcloud2(craters_inverted, header.stamp, 'base_link')
            )

    def numpy_to_pointcloud2(self, points, timestamp, frame_id):
        """Convert numpy (N x 3) array to PointCloud2 message."""
        msg = PointCloud2()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id

        if points.shape[0] == 0:
            return msg

        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True

        msg.data = points.astype(np.float32).tobytes()

        return msg

    def print_statistics(self, roi_count, rock_count, crater_count, obstacle_count):
        """Print processing statistics every 5 seconds."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Perception Statistics (last 5 seconds):')
        self.get_logger().info(f'  Frames processed: {self.frame_count}')
        self.get_logger().info(f'  Points in ROI: {roi_count}')
        self.get_logger().info(f'  Rocks detected: {rock_count}')
        self.get_logger().info(f'  Craters inverted: {crater_count}')
        self.get_logger().info(f'  Total obstacles: {obstacle_count}')
        self.get_logger().info('=' * 60)

        self.frame_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
