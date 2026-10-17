#!/usr/bin/env python3
"""Small online DQN example for scan-only local obstacle avoidance.

This node is intentionally local: a LaserScan does not contain a navigation
goal or a global pose, so arbitrary goal navigation remains the responsibility
of Nav2.
"""

from collections import deque
import random
from typing import Deque, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import torch
    from torch import nn
except ImportError as exc:
    raise RuntimeError(
        'dqn_nav_node.py requires PyTorch. Install it with '
        '`python3 -m pip install --user torch` or your platform package.'
    ) from exc


Transition = Tuple[np.ndarray, int, float, np.ndarray, bool]


class QNetwork(nn.Module):
    def __init__(self, state_size: int, action_size: int) -> None:
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(state_size, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, action_size),
        )

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        return self.layers(state)


class DqnNavNode(Node):
    ACTIONS = (
        (0.18, 0.0),
        (0.10, 0.8),
        (0.10, -0.8),
        (0.0, 1.2),
        (0.0, -1.2),
    )

    def __init__(self) -> None:
        super().__init__('dqn_nav_node')
        self.declare_parameter('epsilon_start', 1.0)
        self.declare_parameter('epsilon_min', 0.05)
        self.declare_parameter('epsilon_decay', 0.995)
        self.declare_parameter('learning_rate', 1e-3)
        self.declare_parameter('gamma', 0.95)
        self.declare_parameter('batch_size', 64)
        self.declare_parameter('replay_capacity', 10000)

        self.state_size = 24
        self.action_size = len(self.ACTIONS)
        self.device = torch.device('cpu')
        self.policy_net = QNetwork(self.state_size, self.action_size).to(self.device)
        self.target_net = QNetwork(self.state_size, self.action_size).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.optimizer = torch.optim.Adam(
            self.policy_net.parameters(),
            lr=float(self.get_parameter('learning_rate').value))
        self.replay: Deque[Transition] = deque(
            maxlen=int(self.get_parameter('replay_capacity').value))
        self.gamma = float(self.get_parameter('gamma').value)
        self.batch_size = int(self.get_parameter('batch_size').value)
        self.epsilon = float(self.get_parameter('epsilon_start').value)
        self.epsilon_min = float(self.get_parameter('epsilon_min').value)
        self.epsilon_decay = float(self.get_parameter('epsilon_decay').value)
        self.previous_state: Optional[np.ndarray] = None
        self.previous_action: Optional[int] = None
        self.training_steps = 0

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info(
            'DQN local obstacle-avoidance node started; epsilon=%.2f',
            self.epsilon)

    def scan_callback(self, message: LaserScan) -> None:
        state = self.scan_to_state(message)
        minimum_range = float(np.min(state))
        collision = minimum_range < 0.12

        if self.previous_state is not None and self.previous_action is not None:
            reward = -1.0 if collision else 0.05 * minimum_range
            self.replay.append((
                self.previous_state, self.previous_action, reward, state, collision))
            self.train_step()

        action = self.select_action(state)
        self.publish_action(action)
        self.previous_state = state
        self.previous_action = action

        if collision:
            self.publish_stop()
            self.previous_state = None
            self.previous_action = None

    def scan_to_state(self, message: LaserScan) -> np.ndarray:
        ranges = np.asarray(message.ranges, dtype=np.float32)
        ranges = np.nan_to_num(
            ranges,
            nan=message.range_max,
            posinf=message.range_max,
            neginf=message.range_min)
        ranges = np.clip(ranges, message.range_min, message.range_max)
        indices = np.linspace(0, len(ranges) - 1, self.state_size).astype(int)
        return np.clip(ranges[indices] / message.range_max, 0.0, 1.0)

    def select_action(self, state: np.ndarray) -> int:
        if random.random() < self.epsilon:
            action = random.randrange(self.action_size)
        else:
            with torch.no_grad():
                tensor = torch.from_numpy(state).float().unsqueeze(0).to(self.device)
                action = int(torch.argmax(self.policy_net(tensor), dim=1).item())
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        return action

    def train_step(self) -> None:
        if len(self.replay) < self.batch_size:
            return
        batch: List[Transition] = random.sample(self.replay, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        state_tensor = torch.from_numpy(np.stack(states)).float()
        action_tensor = torch.tensor(actions, dtype=torch.long).unsqueeze(1)
        reward_tensor = torch.tensor(rewards, dtype=torch.float32)
        next_state_tensor = torch.from_numpy(np.stack(next_states)).float()
        done_tensor = torch.tensor(dones, dtype=torch.float32)

        current_q = self.policy_net(state_tensor).gather(1, action_tensor).squeeze(1)
        with torch.no_grad():
            next_q = self.target_net(next_state_tensor).max(dim=1).values
            expected_q = reward_tensor + self.gamma * next_q * (1.0 - done_tensor)
        loss = nn.functional.smooth_l1_loss(current_q, expected_q)
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.policy_net.parameters(), 1.0)
        self.optimizer.step()

        self.training_steps += 1
        if self.training_steps % 100 == 0:
            self.target_net.load_state_dict(self.policy_net.state_dict())

    def publish_action(self, action: int) -> None:
        message = Twist()
        message.linear.x, message.angular.z = self.ACTIONS[action]
        self.cmd_vel_publisher.publish(message)

    def publish_stop(self) -> None:
        self.cmd_vel_publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DqnNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
