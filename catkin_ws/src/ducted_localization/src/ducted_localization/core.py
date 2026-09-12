"""ROS-independent geometry and registration acceptance policy."""
import math
import numpy as np


def valid_transform(matrix):
    m = np.asarray(matrix, dtype=float)
    return (m.shape == (4, 4) and np.isfinite(m).all()
            and np.allclose(m[3], [0, 0, 0, 1], atol=1e-7)
            and np.allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=1e-5)
            and abs(np.linalg.det(m[:3, :3]) - 1.) < 1e-5)


def pose_matrix(position, quaternion):
    p, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('non-finite or malformed pose')
    norm = np.linalg.norm(q)
    if abs(norm - 1.) > .01: raise ValueError('pose quaternion is not normalized')
    x, y, z, w = q / norm
    m = np.eye(4)
    m[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    m[:3, 3] = p
    return m


def initial_correction(map_base, local_body, base_body):
    if not all(valid_transform(m) for m in (map_base, local_body, base_body)):
        raise ValueError('invalid manual initialization transform')
    return map_base @ base_body @ np.linalg.inv(local_body)


def consistent(a, b, translation=.5, rotation=.35):
    delta = np.linalg.inv(a) @ b
    angle = math.acos(float(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1)))
    return np.linalg.norm(delta[:3, 3]) <= translation and angle <= rotation


class Session:
    """Manual commands replace generations; a lost session never restarts itself."""
    def __init__(self, timeout=60., min_fitness=.55, max_rmse=.30,
                 required_confirmations=2, tracking_timeout=3., failure_limit=3,
                 max_translation=.5, max_rotation=.35):
        values = (timeout, min_fitness, max_rmse, tracking_timeout, max_translation, max_rotation)
        if not all(math.isfinite(x) and x > 0 for x in values) or min_fitness > 1:
            raise ValueError('invalid registration limits')
        if required_confirmations < 2 or failure_limit < 1: raise ValueError('invalid confirmation count')
        self.timeout, self.min_fitness, self.max_rmse = timeout, min_fitness, max_rmse
        self.required = int(required_confirmations)
        self.tracking_timeout, self.failure_limit = tracking_timeout, int(failure_limit)
        self.max_translation, self.max_rotation = max_translation, max_rotation
        self.generation = 0
        self.state, self.reason, self.source = 'WAITING', 'waiting for initialization', 'NONE'
        self.transform = self.candidate = self.seed = None
        self.confirmations = self.failures = 0
        self.last_job = -1
        self.started = self.last_good = None
        self.fitness = self.rmse = None

    def begin(self, source, now, seed=None):
        if source not in ('AUTO', 'MANUAL') or not math.isfinite(now): raise ValueError('invalid session')
        if source == 'MANUAL' and (seed is None or not valid_transform(seed)):
            raise ValueError('manual initialization requires a valid seed')
        self.generation += 1
        self.source, self.state, self.reason = source, 'SEARCHING', 'collecting fresh scans'
        self.started, self.last_good = now, None
        self.seed = None if seed is None else seed.copy()
        self.candidate = self.transform = None
        self.confirmations = self.failures = 0
        self.last_job = -1
        self.fitness = self.rmse = None
        return self.generation

    def invalidate(self, reason, state='LOST'):
        self.generation += 1
        self.state, self.reason = state, reason
        self.transform = None

    def tick(self, now):
        if self.started is not None and now < self.started:
            self.invalidate('monotonic clock moved backwards')
        elif self.state == 'SEARCHING' and now - self.started > self.timeout:
            self.invalidate('initialization timed out; use RViz or retry service', 'FAILED')
        elif self.state == 'TRACKING' and now - self.last_good > self.tracking_timeout:
            self.invalidate('map tracking expired; explicit reinitialization required')

    def fresh(self, now):
        return (self.state == 'TRACKING' and self.transform is not None
                and self.last_good is not None and 0 <= now-self.last_good <= self.tracking_timeout)

    def reject(self, reason):
        self.reason = reason
        if self.state == 'TRACKING':
            self.failures += 1
            if self.failures >= self.failure_limit: self.invalidate(reason)
        else:
            self.confirmations = 0
            self.candidate = None

    def accept(self, generation, job_id, transform, fitness, rmse, now):
        self.tick(now)
        if generation != self.generation or job_id <= self.last_job or self.state not in ('SEARCHING', 'TRACKING'):
            return False
        self.last_job = job_id
        if (not valid_transform(transform) or not math.isfinite(fitness) or not math.isfinite(rmse)
                or not self.min_fitness <= fitness <= 1 or not 0 <= rmse <= self.max_rmse):
            self.reject('registration quality rejected')
            return False
        previous = self.transform if self.state == 'TRACKING' else self.candidate
        if previous is not None and not consistent(previous, transform, self.max_translation, self.max_rotation):
            self.reject('independent registrations disagree or correction jump rejected')
            return False
        self.fitness, self.rmse = float(fitness), float(rmse)
        self.candidate = transform.copy()
        if self.state == 'SEARCHING':
            self.confirmations += 1
            if self.confirmations < self.required:
                self.reason = 'waiting for independent confirmation'
                return False
        self.transform = transform.copy()
        self.last_good, self.failures = now, 0
        self.state, self.reason = 'TRACKING', 'registration accepted'
        return True
