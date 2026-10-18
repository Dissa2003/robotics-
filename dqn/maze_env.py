"""Discrete 6x6 maze derived from wall collisions in worlds/maze.world.
Each legal action moves one 0.5m grid cell; NOT a physics or LiDAR simulator.
"""
from collections import deque
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / 'worlds/maze.world'
N = 6
STEP = 0.5
ACTION_NAMES = ('up', 'right', 'down', 'left')
DELTA = ((-1, 0), (0, 1), (1, 0), (0, -1))


def read_walls(world=WORLD):
    root = ET.parse(world).getroot()
    result = []
    for model in root.findall('.//world/model'):
        if not model.get('name', '').startswith('wall_'):
            continue
        pose = [float(x) for x in model.findtext('pose').split()]
        size = [float(x) for x in model.findtext('./link/collision/geometry/box/size').split()]
        result.append((pose[0], pose[1], size[0], size[1]))
    if not result:
        raise ValueError('No wall models found in maze world.')
    return result


def center(cell):
    row, col = divmod(cell, N)
    return (-1.25 + STEP * col, 1.25 - STEP * row)


class Maze:
    def __init__(self, world=WORLD):
        self.walls = read_walls(world)
        self.edges = {}
        for cell in range(N * N):
            r, c = divmod(cell, N)
            x0, y0 = center(cell)
            moves = {}
            for action, (dr, dc) in enumerate(DELTA):
                nr, nc = r + dr, c + dc
                if not (0 <= nr < N and 0 <= nc < N):
                    continue
                nxt = nr * N + nc
                x1, y1 = center(nxt)
                mx, my = (x0 + x1) / 2, (y0 + y1) / 2
                blocked = any(abs(mx - wx) <= sx / 2 + 1e-5 and
                              abs(my - wy) <= sy / 2 + 1e-5
                              for wx, wy, sx, sy in self.walls)
                if not blocked:
                    moves[action] = nxt
            self.edges[cell] = moves
        reachable = len(self.distances_from(0))
        if reachable != N * N:
            raise ValueError(f'Maze has {reachable} of {N * N} reachable cells; check wall coordinates')

    def distances_from(self, goal):
        distance = {goal: 0}
        queue = deque([goal])
        while queue:
            cell = queue.popleft()
            for nxt in self.edges[cell].values():
                if nxt not in distance:
                    distance[nxt] = distance[cell] + 1
                    queue.append(nxt)
        return distance

    def shortest_path(self, start, goal):
        queue = deque([start])
        parents = {start: None}
        while queue:
            cell = queue.popleft()
            if cell == goal:
                break
            for nxt in self.edges[cell].values():
                if nxt not in parents:
                    parents[nxt] = cell
                    queue.append(nxt)
        if goal not in parents:
            return None
        result = [goal]
        while result[-1] != start:
            result.append(parents[result[-1]])
        return list(reversed(result))

    @staticmethod
    def features(cell, goal):
        v = np.zeros(N * N * 2, dtype=np.float32)
        v[cell] = 1.
        v[N * N + goal] = 1.
        return v


class GridMazeEnv:
    def __init__(self, maze=None, max_steps=70):
        self.maze = maze or Maze()
        self.max_steps = max_steps
        self.cell = 0
        self.goal = 35
        self.steps = 0
        self.bump_count = 0
        self.distances = self.maze.distances_from(self.goal)

    def reset(self, cell=0, goal=35):
        if not (0 <= cell < 36 and 0 <= goal < 36):
            raise ValueError('start and goal must be in [0, 35]')
        self.cell = cell
        self.goal = goal
        self.steps = 0
        self.bump_count = 0
        self.distances = self.maze.distances_from(goal)
        return self.maze.features(cell, goal)

    def step(self, action):
        self.steps += 1
        previous = self.cell
        target = self.maze.edges[self.cell].get(int(action), self.cell)
        collided = target == previous
        if collided:
            self.bump_count += 1
        self.cell = target
        succeeded = target == self.goal
        expired = self.steps >= self.max_steps
        progress = self.distances[previous] - self.distances[target]
        reward = -0.04 + 0.22 * progress
        if collided:
            reward -= 0.65
        if succeeded:
            reward += 8.0
        return self.maze.features(target, self.goal), reward, succeeded or expired, {
            'success': succeeded,
            'invalid_moves': self.bump_count,
            'cell': self.cell,
        }
