"""
PSO Engine - Particle Swarm Optimization for traffic signal timing.
Each particle represents a configuration of green-phase durations
across all active junctions. The swarm minimizes total vehicle
waiting time (approximated by congestion on junction edges).
"""
import numpy as np
from collections import defaultdict

# PSO Hyperparameters
N_PARTICLES = 15
MAX_ITER = 10          # iterations per timestep (keep low for real-time)
W = 0.5                # inertia weight
C1 = 1.5               # cognitive coefficient (personal best)
C2 = 1.5               # social coefficient (global best)
PHASE_MIN = 10.0       # minimum green phase duration (seconds)
PHASE_MAX = 60.0       # maximum green phase duration (seconds)


class PSOEngine:
    def __init__(self):
        self.junctions = []          # list of junction ids
        self.particles = None        # shape: (N_PARTICLES, n_junctions)
        self.velocities = None
        self.personal_best = None
        self.personal_best_score = None
        self.global_best = None
        self.global_best_score = float('inf')
        # junction_congestion[junction_id] -> total vehicles on adjacent edges
        self.junction_congestion = defaultdict(float)
        self.history = []            # score history for visualization

    def set_junctions(self, junction_ids: list[str]):
        """Initialize swarm for a given set of junctions."""
        if set(junction_ids) == set(self.junctions) and self.particles is not None:
            return  # no reinit needed
        self.junctions = junction_ids
        n = len(junction_ids)
        if n == 0:
            return
        self.particles = np.random.uniform(PHASE_MIN, PHASE_MAX, (N_PARTICLES, n))
        self.velocities = np.random.uniform(-5, 5, (N_PARTICLES, n))
        self.personal_best = self.particles.copy()
        self.personal_best_score = np.full(N_PARTICLES, float('inf'))
        self.global_best = self.particles[0].copy()
        self.global_best_score = float('inf')

    def update_congestion(self, vehicles: list[dict], lane_to_junction: dict):
        """
        Update junction congestion from current vehicle states.
        lane_to_junction maps lane_id -> junction_id.
        """
        self.junction_congestion.clear()
        for v in vehicles:
            lane = v.get('lane', '')
            junction = lane_to_junction.get(lane)
            if junction:
                self.junction_congestion[junction] += 1

    def _fitness(self, particle: np.ndarray) -> float:
        """
        Fitness = total weighted waiting time across junctions.
        Longer green phases reduce congestion but must be balanced.
        """
        if not self.junctions:
            return 0.0
        total_cost = 0.0
        for i, junc_id in enumerate(self.junctions):
            congestion = self.junction_congestion.get(junc_id, 0)
            green_time = particle[i]
            # vehicles waiting = congestion / (green_time / cycle_time)
            # simplified: cost = congestion * (PHASE_MAX - green_time) / PHASE_MAX
            cycle_efficiency = green_time / PHASE_MAX
            waiting = congestion * (1.0 - cycle_efficiency)
            total_cost += waiting
        return total_cost

    def optimize(self, vehicles: list[dict], lane_to_junction: dict) -> dict:
        """
        Run PSO optimization cycle.
        Returns optimal signal timings per junction.
        """
        self.update_congestion(vehicles, lane_to_junction)

        if not self.junctions or self.particles is None:
            return {}

        for _ in range(MAX_ITER):
            for i in range(N_PARTICLES):
                score = self._fitness(self.particles[i])

                # Update personal best
                if score < self.personal_best_score[i]:
                    self.personal_best_score[i] = score
                    self.personal_best[i] = self.particles[i].copy()

                # Update global best
                if score < self.global_best_score:
                    self.global_best_score = score
                    self.global_best = self.particles[i].copy()

            # Update velocities and positions
            r1 = np.random.rand(N_PARTICLES, len(self.junctions))
            r2 = np.random.rand(N_PARTICLES, len(self.junctions))

            self.velocities = (
                W * self.velocities
                + C1 * r1 * (self.personal_best - self.particles)
                + C2 * r2 * (self.global_best - self.particles)
            )
            self.particles = np.clip(
                self.particles + self.velocities,
                PHASE_MIN, PHASE_MAX
            )

        self.history.append(round(self.global_best_score, 3))
        if len(self.history) > 100:
            self.history.pop(0)

        # Build result dict
        result = {}
        for i, junc_id in enumerate(self.junctions):
            result[junc_id] = {
                "green_phase_duration": round(float(self.global_best[i]), 2),
                "congestion": self.junction_congestion.get(junc_id, 0)
            }
        return result

    def get_convergence(self) -> list[float]:
        return self.history
