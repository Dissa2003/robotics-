"""Small fully-connected Deep Q Network implemented with NumPy (CPU/ARM friendly).
Two trainable hidden ReLU layers, replay minibatches, target network, Adam updates.
"""
from pathlib import Path
import numpy as np


class QNetwork:
    def __init__(self, seed=42, hidden=64):
        rng = np.random.default_rng(seed)
        self.params = {
            'w1': (rng.normal(0, 1, (72, hidden)) * np.sqrt(2 / 72)).astype(np.float32),
            'b1': np.zeros(hidden, dtype=np.float32),
            'w2': (rng.normal(0, 1, (hidden, hidden)) * np.sqrt(2 / hidden)).astype(np.float32),
            'b2': np.zeros(hidden, dtype=np.float32),
            'w3': (rng.normal(0, 1, (hidden, 4)) * np.sqrt(1 / hidden)).astype(np.float32),
            'b3': np.zeros(4, dtype=np.float32),
        }
        self.m = {k: np.zeros_like(v) for k, v in self.params.items()}
        self.v = {k: np.zeros_like(v) for k, v in self.params.items()}
        self.t = 0

    def predict(self, x):
        p = self.params
        x = np.asarray(x, np.float32)
        h1 = np.maximum(0, x @ p['w1'] + p['b1'])
        h2 = np.maximum(0, h1 @ p['w2'] + p['b2'])
        return h2 @ p['w3'] + p['b3']

    def train_batch(self, x, actions, targets, lr=0.0005):
        p = self.params
        z1 = x @ p['w1'] + p['b1']
        h1 = np.maximum(0, z1)
        z2 = h1 @ p['w2'] + p['b2']
        h2 = np.maximum(0, z2)
        q = h2 @ p['w3'] + p['b3']
        batch = np.arange(len(actions))
        error = q[batch, actions] - targets
        # Huber loss derivative (stable TD-error gradients).
        dq = np.zeros_like(q)
        dq[batch, actions] = np.clip(error, -1.0, 1.0) / len(actions)
        grads = {}
        grads['w3'] = h2.T @ dq
        grads['b3'] = dq.sum(axis=0)
        dh2 = (dq @ p['w3'].T) * (z2 > 0)
        grads['w2'] = h1.T @ dh2
        grads['b2'] = dh2.sum(axis=0)
        dh1 = (dh2 @ p['w2'].T) * (z1 > 0)
        grads['w1'] = x.T @ dh1
        grads['b1'] = dh1.sum(axis=0)
        self.t += 1
        for key, g in grads.items():
            self.m[key] = .9 * self.m[key] + .1 * g
            self.v[key] = .999 * self.v[key] + .001 * (g * g)
            m = self.m[key] / (1 - .9 ** self.t)
            v = self.v[key] / (1 - .999 ** self.t)
            p[key] -= lr * m / (np.sqrt(v) + 1e-8)
        return float(np.mean(error ** 2))

    def copy_from(self, source):
        self.params = {k: v.copy() for k, v in source.params.items()}

    def save(self, filename):
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(filename, **self.params)

    @classmethod
    def load(cls, filename):
        data = np.load(filename)
        net = cls(hidden=data['w1'].shape[1])
        for k in net.params:
            net.params[k] = data[k].astype(np.float32)
        return net
