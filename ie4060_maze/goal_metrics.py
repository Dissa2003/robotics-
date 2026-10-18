"""Record Nav2 action outcomes and robot odometry path length (Gazebo runs).
ros2 run ie4060_maze goal_metrics --ros-args -p use_sim_time:=true
"""
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray


class MetricsNode(Node):
    def __init__(self):
        super().__init__('ie4060_goal_metrics')
        self.declare_parameter('results_file', str(Path.home() / 'ie4060_ws/results/nav2_runs.csv'))
        self.path = Path(self.get_parameter('results_file').value)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 30)
        self.active_id = None
        self.completed = set()
        self.start_time = 0.0
        self.last_position = None
        self.distance = 0.0
        self.count = 0
        self.get_logger().info('Waiting for RViz Nav2 goals; writing %s' % self.path)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        if self.active_id is None:
            return
        xy = (pos.x, pos.y)
        if self.last_position is not None:
            segment = math.dist(xy, self.last_position)
            if segment < 0.5:  # guard odom teleport/reset between runs
                self.distance += segment
        self.last_position = xy

    def on_status(self, msg):
        for event in msg.status_list:
            key = bytes(event.goal_info.goal_id.uuid).hex()
            status = event.status
            if status in (1, 2, 3) and self.active_id is None and key not in self.completed:
                self.active_id = key
                self.start_time = self.now_s()
                self.last_position = None
                self.distance = 0.0
                self.get_logger().info(f'Goal started: {key[:8]}')
            elif status in (4, 5, 6) and key == self.active_id:
                label = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}[status]
                duration = max(0.0, self.now_s() - self.start_time)
                self.count += 1
                new_file = not self.path.exists()
                with self.path.open('a', newline='') as f:
                    writer = csv.writer(f)
                    if new_file:
                        writer.writerow(['run', 'goal_uuid', 'outcome', 'sim_time_s',
                                         'odom_path_length_m'])
                    writer.writerow([self.count, key, label, f'{duration:.3f}',
                                     f'{self.distance:.3f}'])
                self.get_logger().info(
                    f'Goal {label}: sim time {duration:.1f}s, traveled {self.distance:.2f}m')
                self.completed.add(key)
                self.active_id = None
                self.last_position = None


def main():
    rclpy.init()
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
