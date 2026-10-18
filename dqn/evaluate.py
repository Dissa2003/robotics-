"""Compare DQN vs shortest-path baseline in the same 6x6 GRID simulator.
Grid-step counts are NOT Gazebo wall-clock seconds; keep separate from Nav2 data.
"""
import argparse
import csv
from pathlib import Path
import numpy as np

from .maze_env import GridMazeEnv, Maze
from .model import QNetwork

ROOT = Path(__file__).resolve().parents[1]


def evaluate(model_path, output=None, goals=(5, 17, 35), start=0):
    env = GridMazeEnv(max_steps=70)
    model = QNetwork.load(model_path)
    records = []
    for goal in goals:
        if goal == start:
            continue
        shortest = env.maze.shortest_path(start, goal)
        obs = env.reset(start, goal)
        visited = [start]
        info = {'success': False, 'invalid_moves': 0}
        for step in range(env.max_steps):
            action = int(np.argmax(model.predict(obs)))
            obs, reward, done, info = env.step(action)
            visited.append(env.cell)
            if done:
                break
        traveled = sum(0.5 for a, b in zip(visited, visited[1:]) if a != b)
        records.append({
            'goal_cell': goal, 'method': 'DQN_GRID',
            'success': int(info['success']), 'steps': env.steps,
            'traveled_m': round(traveled, 2),
            'invalid_actions': info['invalid_moves'],
            'optimal_grid_steps': len(shortest) - 1,
            'path_efficiency': round((len(shortest) - 1) / env.steps, 3) if info['success'] else 0.0,
        })
        records.append({
            'goal_cell': goal, 'method': 'SHORTEST_GRID',
            'success': 1, 'steps': len(shortest) - 1,
            'traveled_m': round((len(shortest) - 1) * 0.5, 2),
            'invalid_actions': 0, 'optimal_grid_steps': len(shortest) - 1,
            'path_efficiency': 1.0,
        })
    outfile = Path(output or ROOT / 'results/grid_comparison.csv')
    outfile.parent.mkdir(parents=True, exist_ok=True)
    with outfile.open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    for r in records:
        print(f"{r['method']:14} goal={r['goal_cell']:2} success={r['success']} "
              f"steps={r['steps']:2} invalid={r['invalid_actions']}")
    print('Saved', outfile)
    return records


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--model', default=str(ROOT / 'dqn/model/dqn_maze.npz'))
    p.add_argument('--output')
    args = p.parse_args()
    evaluate(args.model, args.output)


if __name__ == '__main__':
    main()
