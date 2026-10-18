"""Goal-conditioned DQN learning on a 6x6 discrete maze proxy; NOT Gazebo physics.
Run from repository root: python3 -m dqn.train --episodes 2000
"""
import argparse
from collections import deque
from pathlib import Path
import numpy as np

from .maze_env import GridMazeEnv
from .model import QNetwork

ROOT = Path(__file__).resolve().parents[1]


def train(episodes=2000, seed=7, save=None, log_every=100):
    rng = np.random.default_rng(seed)
    environment = GridMazeEnv(max_steps=65)
    policy = QNetwork(seed=seed)
    target = QNetwork(seed=seed + 1)
    target.copy_from(policy)
    memory = deque(maxlen=30000)
    last_success = deque(maxlen=100)
    iterations = 0

    for episode in range(1, episodes + 1):
        start = int(rng.integers(36))
        goal = int(rng.integers(35))
        if goal >= start:
            goal += 1
        state = environment.reset(cell=start, goal=goal)
        # Hold exploration longer over 36x35 start/goal combinations.
        epsilon = max(0.08, 1.0 - episode / max(episodes * 0.8, 1))
        success = False
        for step in range(environment.max_steps):
            if rng.random() < epsilon:
                action = int(rng.integers(4))
            else:
                action = int(np.argmax(policy.predict(state)))
            next_state, reward, done, info = environment.step(action)
            memory.append((state, action, reward, next_state, float(done)))
            state = next_state
            success = info['success']
            iterations += 1
            if len(memory) >= 128 and iterations % 3 == 0:
                inds = rng.integers(len(memory), size=64)
                batch = [memory[i] for i in inds]
                sx = np.stack([t[0] for t in batch])
                act = np.array([t[1] for t in batch], dtype=np.int32)
                rewards = np.array([t[2] for t in batch], dtype=np.float32)
                nxt = np.stack([t[3] for t in batch])
                ends = np.array([t[4] for t in batch], dtype=np.float32)
                # Double DQN: select using policy, evaluate with target.
                best = np.argmax(policy.predict(nxt), axis=1)
                fut = target.predict(nxt)[np.arange(len(best)), best]
                td_target = rewards + 0.97 * (1 - ends) * fut
                policy.train_batch(sx, act, td_target)
            if iterations % 600 == 0:
                target.copy_from(policy)
            if done:
                break
        last_success.append(float(success))
        if log_every and episode % log_every == 0:
            print(f'episode={episode:4} epsilon={epsilon:.2f} '
                  f'last100_train_success={np.mean(last_success):.1%}', flush=True)
    output = Path(save or ROOT / 'dqn/model/dqn_maze.npz')
    policy.save(output)
    print(f'Model saved to {output}. Evaluate separately on fixed goal tests.')
    return policy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', type=int, default=2000)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--save')
    args = parser.parse_args()
    train(episodes=args.episodes, seed=args.seed, save=args.save)


if __name__ == '__main__':
    main()
