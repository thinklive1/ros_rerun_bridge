#!/root/miniconda3/bin/python
"""
ROS to Rerun Bridge - Python Version (Point Cloud Only)
Supports gRPC connections to remote Rerun viewers

This script subscribes to topics listed in the ROS parameter
`topic_to_datatype` (only sensor_msgs/PointCloud2 supported) and
forwards them to the Rerun viewer under the mapped entity paths
from `topic_to_entity_path`.
"""

import rospy
import rerun as rr
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import functools


class RerunPointCloudBridge:
    def __init__(self):
        rospy.init_node('rerun_pointcloud_bridge', anonymous=False)

        # Read parameters (support private and legacy/global names)
        self.rerun_addr = self._get_param_any(['~rerun_addr', 'rerun_addr'], 'rerun+http://172.17.0.1:9876/proxy')
        self.pointcloud_topic = self._get_param_any(['~pointcloud_topic', 'pointcloud_topic', 'cloud_topic'], '/cloud_registered')
        self.entity_path = self._get_param_any(['~entity_path', 'entity_path'], '/world/fast_livo2/cloud_registered')
        if not self.entity_path.startswith('/'):
            self.entity_path = '/' + self.entity_path

        rospy.loginfo('=' * 60)
        rospy.loginfo('Rerun Point Cloud Bridge (Python)')
        rospy.loginfo('=' * 60)
        rospy.loginfo(f'Rerun Address: {self.rerun_addr}')
        rospy.loginfo(f'Point Cloud Topic (fallback): {self.pointcloud_topic}')
        rospy.loginfo(f'Entity Path (fallback): {self.entity_path}')

        # Initialize Rerun and connect (best-effort). If connect fails we still create subscribers
        try:
            rospy.loginfo('Initializing Rerun...')
            rr.init('ros_pointcloud_bridge', spawn=False)
            rospy.loginfo(f'Connecting to Rerun viewer at {self.rerun_addr} via gRPC...')
            rr.connect_grpc(url=self.rerun_addr)
            rospy.loginfo('✓ Successfully connected to Rerun viewer!')
        except Exception as e:
            rospy.logwarn(f'Rerun connect failed (continuing in offline mode): {e}')

        # statistics
        self.msg_counters = {}
        self.last_log_time = rospy.Time.now()
        # per-message stats printing control
        self.per_message_stats = rospy.get_param('~per_message_stats', True)
        self.stats_throttle = float(rospy.get_param('~stats_throttle', 1.0))  # seconds
        self.sample_point_count = int(rospy.get_param('~sample_point_count', 3))
        self._last_stats_time = {}

        # load topic/entity mappings (try several common param names)
        topic_map = self._get_param_any(['~topic_to_datatype', 'topic_to_datatype', '/topic_to_datatype'], None)
        entity_map = self._get_param_any(['~topic_to_entity_path', 'topic_to_entity_path', '/topic_to_entity_path'], None)

        self.subscribers = []

        if isinstance(topic_map, dict) and len(topic_map) > 0:
            for topic, dtype in topic_map.items():
                # only support PointCloud2 in this bridge
                if isinstance(dtype, str) and dtype.strip() != 'sensor_msgs/PointCloud2':
                    rospy.loginfo(f'Skipping {topic} (datatype {dtype})')
                    continue
                ent = None
                if isinstance(entity_map, dict) and topic in entity_map:
                    ent = entity_map[topic]
                else:
                    safe_name = topic.strip('/').replace('/', '_')
                    ent = f'/world/fast_livo2/{safe_name}'
                if not ent.startswith('/'):
                    ent = '/' + ent
                self.msg_counters[topic] = 0
                cb = functools.partial(self.generic_pointcloud_callback, topic=topic, entity_path=ent)
                sub = rospy.Subscriber(topic, PointCloud2, cb, queue_size=10)
                self.subscribers.append(sub)
                rospy.loginfo(f'Subscribed to {topic} -> Rerun entity {ent}')
        else:
            # fallback single topic
            t = self.pointcloud_topic
            self.msg_counters[t] = 0
            cb = functools.partial(self.generic_pointcloud_callback, topic=t, entity_path=self.entity_path)
            self.subscriber = rospy.Subscriber(t, PointCloud2, cb, queue_size=10)
            rospy.loginfo(f'Fallback subscribe to {t} -> {self.entity_path}')

        # debug: show a sample of params
        try:
            params = rospy.get_param_names()
            rospy.logdebug('ROS params sample: %s' % params[:40])
        except Exception:
            pass

        # heartbeat timer prints counters once per second
        self.debug_timer = rospy.Timer(rospy.Duration(1.0), self._debug_counters)

        rospy.loginfo('Bridge initialized, waiting for point cloud messages...')
        rospy.loginfo('=' * 60)

    def _get_param_any(self, keys, default=None):
        for k in keys:
            try:
                if rospy.has_param(k):
                    return rospy.get_param(k)
            except Exception:
                continue
        return default

    def generic_pointcloud_callback(self, msg, topic, entity_path):
        try:
            self.msg_counters[topic] = self.msg_counters.get(topic, 0) + 1

            field_names = [f.name for f in msg.fields]
            has_rgb = 'rgb' in field_names or 'rgba' in field_names

            points = []
            colors = []
            if has_rgb:
                for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True):
                    x, y, z, rgb = p
                    points.append([x, y, z])
                    try:
                        rgb_int = int(rgb)
                        r = (rgb_int >> 16) & 0xFF
                        g = (rgb_int >> 8) & 0xFF
                        b = rgb_int & 0xFF
                        colors.append([r, g, b])
                    except Exception:
                        colors.append([255, 255, 255])
            else:
                for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                    points.append([p[0], p[1], p[2]])

            if len(points) == 0:
                rospy.logwarn_throttle(5.0, f'Received empty point cloud on {topic}')
                return

            pts_np = np.array(points, dtype=np.float32)

            # Compute and print per-message statistics (throttled)
            try:
                nowt = rospy.Time.now().to_sec()
                last = self._last_stats_time.get(topic, 0.0)
                if self.per_message_stats and (nowt - last >= float(self.stats_throttle)):
                    self._last_stats_time[topic] = nowt
                    cnt = pts_np.shape[0]
                    mins = np.min(pts_np, axis=0).tolist()
                    maxs = np.max(pts_np, axis=0).tolist()
                    means = np.mean(pts_np, axis=0).tolist()
                    stds = np.std(pts_np, axis=0).tolist()
                    p10 = np.percentile(pts_np, 10, axis=0).tolist()
                    p50 = np.percentile(pts_np, 50, axis=0).tolist()
                    p90 = np.percentile(pts_np, 90, axis=0).tolist()
                    sample_n = min(self.sample_point_count, cnt)
                    sample_idx = np.linspace(0, cnt - 1, sample_n, dtype=int) if cnt > 0 else []
                    samples = pts_np[sample_idx].tolist() if cnt > 0 else []
                    rospy.loginfo(f"[PC Stats] topic={topic} count={cnt} min={mins} max={maxs} mean={means} std={stds}")
                    rospy.loginfo(f"[PC Stats] p10={p10} p50={p50} p90={p90} samples={samples}")
            except Exception:
                rospy.logdebug('Failed to compute/print pointcloud stats')

            timestamp_sec = msg.header.stamp.to_sec()
            try:
                # optional, set Rerun time if available
                if hasattr(rr, 'set_time_seconds'):
                    rr.set_time_seconds('ros_time', timestamp_sec)
            except Exception:
                pass

            try:
                if has_rgb and len(colors) > 0:
                    colors_np = np.array(colors, dtype=np.uint8)
                    rr.log(entity_path, rr.Points3D(pts_np, colors=colors_np))
                else:
                    rr.log(entity_path, rr.Points3D(pts_np))
                rospy.logdebug(f'rr.log called for {entity_path} (topic {topic}) pts={len(points)}')
            except Exception as e:
                rospy.logerr(f'Rerun logging failed for {entity_path} from topic {topic}: {e}')

            now = rospy.Time.now()
            if (now - self.last_log_time).to_sec() >= 1.0:
                total = sum(self.msg_counters.values())
                rospy.loginfo(f'[Stats] total_msgs={total} latest_topic={topic} topic_msgs={self.msg_counters[topic]} pts_in_msg={len(points)} frame={msg.header.frame_id} seq={msg.header.seq} time={timestamp_sec:.3f} -> sent to {entity_path}')
                self.last_log_time = now

        except Exception as e:
            rospy.logerr(f'Error processing point cloud from {topic}: {e}')
            import traceback
            rospy.logerr(traceback.format_exc())

    def _debug_counters(self, event):
        try:
            if not self.msg_counters:
                rospy.logdebug('No pointcloud topics registered yet')
                return
            counters = ', '.join([f'{t}:{c}' for t, c in self.msg_counters.items()])
            rospy.loginfo(f'[Heartbeat] pointcloud counters: {counters}')
        except Exception:
            pass

    def run(self):
        rospy.loginfo('Bridge is running. Press Ctrl+C to stop.')
        rospy.spin()


def main():
    try:
        bridge = RerunPointCloudBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down Rerun bridge...')
    except Exception as e:
        rospy.logerr(f'Fatal error: {e}')
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()
