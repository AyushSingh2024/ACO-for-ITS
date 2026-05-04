"""
Traffic Engine - Replays trajectories.csv as a live feed and
runs ACO + PSO at each timestep, simulating real-time optimization.
"""
import os
import sys
import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET
from collections import defaultdict

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'optimization'))
from aco_engine import ACOEngine
from pso_engine import PSOEngine


class TrafficEngine:
    def __init__(self):
        self.aco = ACOEngine()
        self.pso = PSOEngine()

        self.min_time = 0
        self.max_time = 0
        self.x_min = self.x_max = self.y_min = self.y_max = 0
        self.edge_lengths = {}        # edge_id -> length in meters
        self.edge_geoms = {}          # edge_id -> ((startX, startY), (endX, endY))
        self.edge_shapes = {}         # edge_id -> list of (x, y) tuples (full polyline)
        self.lane_to_junction = {}    # lane_id -> junction_id
        self.true_routes = {}         # veh_id -> list of edge_ids
        self.routes_loaded = False
        self._last_aco_result = {}
        self._last_pso_result = {}

    # ------------------------------------------------------------------ #
    #  Data Loading
    # ------------------------------------------------------------------ #

    def load_trajectories(self, csv_path: str):
        if not os.path.exists(csv_path):
            print(f"WARNING: {csv_path} not found")
            return
        df = pd.read_csv(csv_path, sep=';', usecols=['timestep_time', 'vehicle_id', 'vehicle_x', 'vehicle_y', 'vehicle_lane', 'vehicle_speed'])
        req = ['timestep_time', 'vehicle_id', 'vehicle_x', 'vehicle_y',
               'vehicle_lane', 'vehicle_speed']
        if any(c not in df.columns for c in req):
            print("WARNING: Missing columns in trajectories.csv")
            return
        df = df.fillna(0)
        self.min_time = float(df['timestep_time'].min())
        self.max_time = float(df['timestep_time'].max())
        self.x_min = float(df['vehicle_x'].min())
        self.x_max = float(df['vehicle_x'].max())
        self.y_min = float(df['vehicle_y'].min())
        self.y_max = float(df['vehicle_y'].max())
        
        self.available_times = np.sort(df['timestep_time'].unique())
        self.trajectories = df.set_index('timestep_time').sort_index()
        print(f"Loaded {len(df)} trajectory records.")

    def load_network(self, net_path: str):
        """Parse network XML to extract edge lengths and junction mappings."""
        if not os.path.exists(net_path):
            print(f"WARNING: {net_path} not found")
            return
        try:
            tree = ET.parse(net_path)
            root = tree.getroot()
            # Extract edge lengths from lanes
            for edge in root.findall('edge'):
                edge_id = edge.get('id', '')
                if edge_id.startswith(':'):
                    continue  # skip internal edges
                for lane in edge.findall('lane'):
                    length = float(lane.get('length', 100.0))
                    self.edge_lengths[edge_id] = length
                    shape_str = lane.get('shape', '')
                    coords = shape_str.split(' ')
                    if len(coords) >= 2:
                        try:
                            start = tuple(map(float, coords[0].split(',')))
                            end = tuple(map(float, coords[-1].split(',')))
                            self.edge_geoms[edge_id] = (start, end)
                            # Store full polyline shape for map rendering
                            full_shape = []
                            for c in coords:
                                parts = c.split(',')
                                if len(parts) >= 2:
                                    full_shape.append((float(parts[0]), float(parts[1])))
                            if full_shape:
                                self.edge_shapes[edge_id] = full_shape
                        except ValueError:
                            pass
                    break  # one lane per edge is enough

            # Build lane -> junction mapping from connections
            for conn in root.findall('connection'):
                from_edge = conn.get('from', '')
                via = conn.get('via', '')
                # via lane belongs to an internal junction edge ":junction_id_..."
                if via and via.startswith(':'):
                    junction_id = via.split('_')[0][1:]  # strip leading ':'
                    from_lane = f"{from_edge}_{conn.get('fromLane', '0')}"
                    self.lane_to_junction[from_lane] = junction_id

            print(f"Network loaded: {len(self.edge_lengths)} edges, "
                  f"{len(self.lane_to_junction)} lane-junction mappings.")
        except Exception as e:
            print(f"WARNING: Could not parse network: {e}")

    def load_routes(self, routes_path: str):
        """Parse routes XML to build ACO graph."""
        if not os.path.exists(routes_path):
            print(f"WARNING: {routes_path} not found")
            return
        try:
            tree = ET.parse(routes_path)
            root = tree.getroot()
            all_routes = []
            for vehicle in root.findall('vehicle'):
                veh_id = vehicle.get('id')
                route_el = vehicle.find('route')
                if route_el is not None:
                    edges = route_el.get('edges', '').split()
                    if edges:
                        all_routes.append(edges)
                        self.true_routes[veh_id] = edges
            self.aco.build_graph_from_routes(all_routes, self.edge_lengths, self.edge_geoms)
            self.routes_loaded = True
            print(f"ACO graph built from {len(all_routes)} routes.")
        except Exception as e:
            print(f"WARNING: Could not parse routes: {e}")

    # ------------------------------------------------------------------ #
    #  Per-Timestep Processing
    # ------------------------------------------------------------------ #

    def get_vehicles_at(self, timestep: float):

        if not hasattr(self, 'available_times') or len(self.available_times) == 0:
            return [], None
            
        idx = np.searchsorted(self.available_times, timestep)
        if idx == 0:
            closest_t = self.available_times[0]
        elif idx == len(self.available_times):
            closest_t = self.available_times[-1]
        else:
            before = self.available_times[idx - 1]
            after = self.available_times[idx]
            closest_t = float(before if timestep - before < after - timestep else after)

        try:
            vehicles_df = self.trajectories.loc[closest_t]
            if isinstance(vehicles_df, pd.Series):
                vehicles_df = vehicles_df.to_frame().T
            raw = vehicles_df.reset_index().to_dict('records')
        except KeyError:
            raw = []
            
        lane_counts = defaultdict(int)
        for v in raw:
            lane_counts[str(v.get('vehicle_lane', ''))] += 1

        vehicles = []
        for v in raw:
            lane = str(v.get('vehicle_lane', ''))
            count = lane_counts[lane]
            congestion = "High" if count > 5 else "Medium" if count > 2 else "Low"
            vehicles.append({
                "id": v.get('vehicle_id', 'unknown'),
                "x": float(v.get('vehicle_x', 0)),
                "y": float(v.get('vehicle_y', 0)),
                "speed": float(v.get('vehicle_speed', 0)),
                "lane": lane,
                "lane_density": count,
                "congestion": congestion
            })
        return vehicles, closest_t

    def run_optimization(self, timestep: float) -> dict:
        """
        Run ACO + PSO for the given timestep.
        Returns combined optimization results.
        """
        vehicles, _ = self.get_vehicles_at(timestep)
        if not vehicles:
            return {"aco": {}, "pso": {}, "vehicles": []}

        # --- ACO ---
        self.aco.update_congestion(vehicles)
        # Sample a few OD pairs from vehicles currently on the road
        od_pairs = self._sample_od_pairs(vehicles)
        aco_result = self.aco.optimize(od_pairs)
        self._last_aco_result = aco_result

        # --- PSO ---
        active_junctions = list({
            j for lane, j in self.lane_to_junction.items()
            if any(v['lane'] == lane for v in vehicles)
        })[:20]  # cap to 20 junctions for performance
        if not active_junctions:
            # fallback: use all known junctions (sample)
            active_junctions = list(set(self.lane_to_junction.values()))[:20]
        self.pso.set_junctions(active_junctions)
        pso_result = self.pso.optimize(vehicles, self.lane_to_junction)
        self._last_pso_result = pso_result

        return {
            "timestep": timestep,
            "vehicle_count": len(vehicles),
            "aco": {
                "routes_optimized": len(aco_result),
                "top_pheromone_edges": self.aco.get_pheromone_snapshot(10),
                "routes": [
                    {
                        "origin": k[0],
                        "destination": k[1],
                        "estimated_time": v["estimated_time"],
                        "edges_count": v["edges_count"]
                    }
                    for k, v in list(aco_result.items())[:5]
                ]
            },
            "pso": {
                "junctions_optimized": len(pso_result),
                "convergence_score": self.pso.global_best_score
                    if self.pso.global_best_score != float('inf') else 0,
                "convergence_history": self.pso.get_convergence()[-20:],
                "top_junctions": [
                    {
                        "junction": jid,
                        "green_phase": data["green_phase_duration"],
                        "congestion": data["congestion"]
                    }
                    for jid, data in list(pso_result.items())[:5]
                ]
            }
        }

    def _sample_od_pairs(self, vehicles: list[dict]) -> list[tuple[str, str]]:
        """Sample origin-destination edge pairs from current vehicles."""
        edges = []
        for v in vehicles:
            lane = v.get('lane', '')
            edge = '_'.join(lane.split('_')[:-1]) if '_' in lane else lane
            if edge and not edge.startswith(':'):
                edges.append(edge)
        edges = list(set(edges))
        pairs = []
        for i in range(0, min(len(edges) - 1, 10), 2):
            pairs.append((edges[i], edges[i + 1]))
        return pairs

    def get_route_for_driver(self, origin_edge: str, dest_edge: str) -> dict:
        """Find best route for specific driver without altering active simulation state."""
        best_path = None
        best_cost = float('inf')
        # Use a few ants to explore the current pheromone landscape
        for _ in range(20):
            path = self.aco._ant_walk(origin_edge, dest_edge)
            cost = sum(self.aco._travel_time(e) for e in path)
            if cost < best_cost:
                best_cost = cost
                best_path = path

        if best_path:
            return {
                "path": best_path,
                "estimated_time": best_cost,
                "edges_count": len(best_path)
            }
        return None

