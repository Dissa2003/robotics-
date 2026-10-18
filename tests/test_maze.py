import unittest
from dqn.maze_env import Maze, GridMazeEnv, center


class TestMaze(unittest.TestCase):
    def setUp(self):
        self.maze = Maze()

    def test_world_has_all_connected_cells(self):
        self.assertEqual(len(self.maze.distances_from(0)), 36)

    def test_every_move_is_reversible(self):
        for cell in self.maze.edges:
            for action, nxt in self.maze.edges[cell].items():
                self.assertIn(cell, self.maze.edges[nxt].values())

    def test_shortest_path(self):
        route = self.maze.shortest_path(0, 35)
        self.assertEqual(route[0], 0)
        self.assertEqual(route[-1], 35)
        self.assertEqual(len(route)-1, 10)

    def test_collision_penalty(self):
        env = GridMazeEnv(self.maze)
        env.reset(0, 35)
        state, reward, done, info = env.step(0)  # north boundary
        self.assertEqual(info['invalid_moves'], 1)
        self.assertEqual(env.cell, 0)
        self.assertLess(reward, 0)


if __name__ == '__main__':
    unittest.main()
