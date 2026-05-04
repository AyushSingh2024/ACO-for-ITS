"""
ACO Engine - Ant Colony Optimization for route selection.
Operates on a road graph built from trajectory data.
Each edge has a pheromone level; ants probabilistically
choose paths favoring low-congestion, high-pheromone edges.
"""
import numpy as np
import random
from collections import defaultdict

# ACO Hyperparameters
ALPHA = 1.0        # pheromone importance
BETA = 2.0         # heuristic (1/travel_time) importance
EVAPORATION = 0.1  # pheromone evaporation rate per timestep
Q = 100.0          # pheromone deposit constant
N_ANTS = 20        # ants per optimization cycle
INIT_PHEROMONE = 1.0


class ACOEngine:
    def __init__(self):
        # pheromone[edge_id] -> float
        self.pheromone = defaultdict(lambda: INIT_PHEROMONE)
        # delta pheromone deposited THIS cycle only (resets each cycle)
        self.pheromone_delta = defaultdict(float)
        # graph[from_edge] -> list of to_edges (adjacency built from routes)
        self.graph = defaultdict(set)
        # edge_length[edge_id] -> float (meters)
        self.edge_length = {}
        # edge_geoms[edge_id] -> ((startX, startY), (endX, endY))
        self.edge_geoms = {}
        # current congestion: edge_id -> vehicle count
        self.edge_congestion = defaultdict(int)
        # best routes found: (origin, dest) -> list of edges
        self.best_routes = {}

    def build_graph_from_routes(self, routes: list[list[str]], edge_lengths: dict, edge_geoms: dict):
        """Build adjacency graph from pre-computed route edge sequences."""
        self.edge_length.update(edge_lengths)
        self.edge_geoms.update(edge_geoms)
        for route_edges in routes:
            for i in range(len(route_edges) - 1):
                self.graph[route_edges[i]].add(route_edges[i + 1])

    def update_congestion(self, vehicles: list[dict]):
        """Update edge congestion counts from current vehicle states."""
        self.edge_congestion.clear()
        for v in vehicles:
            lane = v.get('lane', '')
            # lane format is "edge_id_laneindex", strip lane index
            edge_id = '_'.join(lane.split('_')[:-1]) if '_' in lane else lane
            self.edge_congestion[edge_id] += 1

    def _travel_time(self, edge_id: str) -> float:
        """Heuristic: estimated travel time based on length and congestion."""
        length = self.edge_length.get(edge_id, 100.0)
        congestion = self.edge_congestion.get(edge_id, 0)
        # base speed 13.89 m/s (50 km/h), penalized by congestion
        speed = max(1.0, 13.89 - congestion * 1.5)
        return length / speed

    def _heuristic(self, u: str, goal: str) -> float:
        if u not in self.edge_geoms or goal not in self.edge_geoms:
            return 0.0
        import math
        ux, uy = self.edge_geoms[u][0]
        gx, gy = self.edge_geoms[goal][0]
        return math.sqrt((ux - gx)**2 + (uy - gy)**2) / 13.89

    def _ant_walk(self, start_edge: str, end_edge: str, max_steps: int = 200) -> list[str]:
        """Single ant traversal from start to end edge."""
        path = [start_edge]
        current = start_edge
        visited = {start_edge}

        for _ in range(max_steps):
            if current == end_edge:
                break
            neighbors = list(self.graph.get(current, []))
            neighbors = [n for n in neighbors if n not in visited]
            if not neighbors:
                break

            # Probability based on pheromone and heuristic
            scores = []
            for n in neighbors:
                tau = self.pheromone[n] ** ALPHA
                est_rem = self._travel_time(n) + self._heuristic(n, end_edge)
                eta = (1.0 / max(0.1, est_rem)) ** BETA
                scores.append(tau * eta)

            total = sum(scores)
            if total == 0:
                next_edge = random.choice(neighbors)
            else:
                probs = [s / total for s in scores]
                next_edge = random.choices(neighbors, weights=probs, k=1)[0]

            path.append(next_edge)
            visited.add(next_edge)
            current = next_edge

        return path

    def evaporate(self):
        """Apply pheromone evaporation to all edges."""
        for edge in list(self.pheromone.keys()):
            self.pheromone[edge] *= (1 - EVAPORATION)
            if self.pheromone[edge] < 0.01:
                self.pheromone[edge] = 0.01

    def deposit(self, path: list[str], cost: float):
        """Deposit pheromone on a path inversely proportional to cost."""
        if cost <= 0:
            return
        deposit_amount = Q / cost
        for edge in path:
            self.pheromone[edge] += deposit_amount
            self.pheromone_delta[edge] += deposit_amount  # track this cycle's gain

    def optimize(self, od_pairs: list[tuple[str, str]]) -> dict:
        """
        Run ACO for a set of origin-destination edge pairs.
        Returns best routes found this cycle.
        """
        self.evaporate()
        self.pheromone_delta.clear()  # reset delta at start of each cycle
        results = {}

        for (origin, dest) in od_pairs[:10]:  # cap to avoid blocking
            best_path = None
            best_cost = float('inf')

            for _ in range(N_ANTS):
                path = self._ant_walk(origin, dest)
                cost = sum(self._travel_time(e) for e in path)
                if cost < best_cost:
                    best_cost = cost
                    best_path = path

            if best_path:
                self.deposit(best_path, best_cost)
                results[(origin, dest)] = {
                    "path": best_path,
                    "estimated_time": round(best_cost, 2),
                    "edges_count": len(best_path)
                }
                self.best_routes[(origin, dest)] = best_path

        return results

    def get_pheromone_snapshot(self, top_n: int = 20) -> list[dict]:
        """
        Return top N edges by pheromone delta (gained THIS cycle).
        This reflects which edges ants are actively choosing RIGHT NOW,
        not which edges have accumulated the most over all time.
        Falls back to absolute pheromone if no delta exists yet.
        """
        source = self.pheromone_delta if any(self.pheromone_delta.values()) else self.pheromone
        sorted_edges = sorted(source.items(), key=lambda x: x[1], reverse=True)
        max_val = sorted_edges[0][1] if sorted_edges else 1.0
        return [
            {
                "edge": e,
                "pheromone": round(self.pheromone[e], 3),   # absolute (for display value)
                "intensity": round(v / max_val, 3)           # relative 0-1 (for bar width)
            }
            for e, v in sorted_edges[:top_n]
        ]
