from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
import pandas as pd
import numpy as np
import os
import sys
import random

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'core'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'optimization'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config'))

from traffic_engine import TrafficEngine
from driver_model import DriverModel
from settings import *

app = FastAPI(title="ACO-ITS Server")

# Allow CORS for local frontend testing
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Serve the frontend statically over the same port
frontend_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "frontend")


# Global store for loaded data
class SimData:
    trajectories = None
    max_time = 0
    min_time = 0
    x_min = x_max = y_min = y_max = 0

sim_data = SimData()
traffic_engine = TrafficEngine()
driver_model = DriverModel(traffic_engine)

@app.on_event("startup")
async def load_data():
    base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    traj_path = os.path.join(base_dir, DATA_DIR, "trajectories_full.csv")
    
    print(f"Loading trajectories from: {traj_path}")
    if not os.path.exists(traj_path):
        print("WARNING: trajectories.csv not found!")
        return

    # Load data into traffic engine for ACO + PSO
    base_net = os.path.join(base_dir, DATA_DIR, "network.net.xml")
    base_routes = os.path.join(base_dir, DATA_DIR, "routes.rou.xml")
    traffic_engine.load_trajectories(traj_path)
    traffic_engine.load_network(base_net)
    traffic_engine.load_routes(base_routes)
    
    sim_data.min_time = traffic_engine.min_time
    sim_data.max_time = traffic_engine.max_time
    sim_data.x_min = traffic_engine.x_min
    sim_data.x_max = traffic_engine.x_max
    sim_data.y_min = traffic_engine.y_min
    sim_data.y_max = traffic_engine.y_max
    sim_data.trajectories = True # Indicate loaded
    print("ACO + PSO engines ready.")

@app.get("/api/meta")
async def get_meta():
    """Returns network bounds and time ranges for the frontend canvas."""
    if sim_data.trajectories is None:
        return {"status": "error", "message": "Data not loaded"}
        
    return {
        "time_min": sim_data.min_time,
        "time_max": sim_data.max_time,
        "x_min": sim_data.x_min,
        "x_max": sim_data.x_max,
        "y_min": sim_data.y_min,
        "y_max": sim_data.y_max
    }

@app.get("/api/vehicles")
async def get_vehicles(timestep: float):
    """Returns all vehicle positions at a specific timestep."""
    if sim_data.trajectories is None:
        raise HTTPException(status_code=500, detail="Data not loaded")
        
    formatted_vehicles, closest_t = traffic_engine.get_vehicles_at(timestep)
    if closest_t is None:
         return {"vehicles": []}
        
    return {
        "timestep": closest_t,
        "vehicles": formatted_vehicles
    }

@app.get("/api/optimize")
async def get_optimization(timestep: float):
    """
    Run ACO + PSO for the given timestep and return optimization results.
    ACO finds low-congestion routes; PSO optimizes signal timings.
    """
    if traffic_engine.min_time == 0 and traffic_engine.max_time == 0:
        raise HTTPException(status_code=503, detail="Traffic engine not ready")
    result = traffic_engine.run_optimization(timestep)
    return result


@app.get("/api/driver-advice/{vehicle_id}")
async def get_driver_advice(vehicle_id: str, timestep: float):
    """
    Returns turn-by-turn navigation instructions for a specific vehicle.
    """
    if sim_data.trajectories is None:
        raise HTTPException(status_code=500, detail="Data not loaded")

    vehicles, closest_t = traffic_engine.get_vehicles_at(timestep)
    if closest_t is None:
         return {"instructions": []}
    
    vehicle = next((v for v in vehicles if str(v.get('vehicle_id')) == vehicle_id), None)
    if not vehicle:
        return JSONResponse(status_code=404, content={"message": f"Vehicle {vehicle_id} not found at timestep {closest_t}"})

    lane = str(vehicle.get('vehicle_lane', ''))
    origin_edge = '_'.join(lane.split('_')[:-1]) if '_' in lane else lane
    if origin_edge.startswith(':'):
        return {"instructions": [{"step": 1, "road": origin_edge, "action": "In intersection", "distance_m": 0, "eta_seconds": 1}]}
        
    true_route = traffic_engine.true_routes.get(vehicle_id, [])
    dest_edge = true_route[-1] if true_route else origin_edge
    
    # Run ACO to find optimal remaining path
    best_path = None
    if origin_edge != dest_edge and traffic_engine.routes_loaded:
        current_vehicles, _ = traffic_engine.get_vehicles_at(timestep)
        if current_vehicles:
            traffic_engine.aco.update_congestion(current_vehicles)

        best_cost = float('inf')
        for _ in range(20):  # Run ACO ants
            path = traffic_engine.aco._ant_walk(origin_edge, dest_edge)
            if path and path[-1] == dest_edge:
                cost = sum(traffic_engine.aco._travel_time(e) for e in path)
                if cost < best_cost:
                    best_cost = cost
                    best_path = path

    # If ACO finds a route, use it. Otherwise, fallback to the original historical route.
    if best_path:
        remaining_path = best_path
    else:
        if origin_edge in true_route:
            idx = true_route.index(origin_edge)
            remaining_path = true_route[idx:]
        else:
            remaining_path = true_route

    instructions = driver_model.generate_instructions(remaining_path) if remaining_path else []
    
    return {
        "vehicle_id": vehicle_id,
        "origin": origin_edge,
        "destination": dest_edge,
        "instructions": instructions,
        "path": remaining_path
    }


@app.get("/api/edges")
async def get_edges():
    """Returns all valid (non-internal) edge IDs for origin/destination selection."""
    if not traffic_engine.edge_lengths:
        raise HTTPException(status_code=503, detail="Network not loaded")
    edges = sorted([e for e in traffic_engine.edge_lengths.keys() if not e.startswith(':')])
    return {"edges": edges, "count": len(edges)}


@app.get("/api/random-trip")
async def get_random_trip():
    """Returns a valid connected origin-destination pair based on historical true routes."""
    if not traffic_engine.true_routes:
        raise HTTPException(status_code=503, detail="True routes not loaded")
    
    # Pick a random route that has at least 2 edges
    valid_routes = [route for route in traffic_engine.true_routes.values() if len(route) >= 2]
    if not valid_routes:
        raise HTTPException(status_code=500, detail="No valid routes found")
        
    route = random.choice(valid_routes)
    return {
        "origin": route[0],
        "destination": route[-1]
    }


@app.get("/api/network-geometry")
async def get_network_geometry():
    """Returns full road network geometry (edge polyline shapes) for map rendering."""
    if not traffic_engine.edge_shapes:
        raise HTTPException(status_code=503, detail="Network geometry not loaded")
    
    edges_geo = []
    for edge_id, shape in traffic_engine.edge_shapes.items():
        if edge_id.startswith(':'):
            continue
        edges_geo.append({
            "id": edge_id,
            "shape": shape,  # list of [x, y] tuples
            "length": round(traffic_engine.edge_lengths.get(edge_id, 0), 1)
        })
    
    return {
        "edges": edges_geo,
        "bounds": {
            "x_min": sim_data.x_min,
            "x_max": sim_data.x_max,
            "y_min": sim_data.y_min,
            "y_max": sim_data.y_max
        }
    }


@app.get("/api/route-plan")
async def get_route_plan(origin: str, destination: str, timestep: float = 0):
    """
    Run ACO to find the optimal path from origin edge to destination edge,
    considering real-time congestion at the given timestep.
    Returns the full optimized path with turn-by-turn instructions.
    """
    if not traffic_engine.routes_loaded:
        raise HTTPException(status_code=503, detail="ACO engine not ready")

    if origin not in traffic_engine.edge_lengths:
        raise HTTPException(status_code=400, detail=f"Origin edge '{origin}' not found in network")
    if destination not in traffic_engine.edge_lengths:
        raise HTTPException(status_code=400, detail=f"Destination edge '{destination}' not found in network")
    if origin == destination:
        raise HTTPException(status_code=400, detail="Origin and destination must be different")

    # Update congestion from current vehicles before routing
    vehicles, _ = traffic_engine.get_vehicles_at(timestep)
    if vehicles:
        traffic_engine.aco.update_congestion(vehicles)

    # Run ACO with more ants for a better solution
    best_path = None
    best_cost = float('inf')
    for _ in range(40):
        path = traffic_engine.aco._ant_walk(origin, destination)
        if path and path[-1] == destination:
            cost = sum(traffic_engine.aco._travel_time(e) for e in path)
            if cost < best_cost:
                best_cost = cost
                best_path = path

    if not best_path:
        # Fallback: check if we have a known true route between these edges
        for route in traffic_engine.true_routes.values():
            if origin in route and destination in route:
                idx_start = route.index(origin)
                idx_end = route.index(destination)
                if idx_start < idx_end:
                    best_path = route[idx_start:idx_end+1]
                    best_cost = sum(traffic_engine.aco._travel_time(e) for e in best_path)
                    break

    if not best_path:
        return {"error": "No path found between these edges. They may not be connected in the network."}

    # Generate turn-by-turn instructions
    instructions = driver_model.generate_instructions(best_path)

    # Compute per-edge speed estimates
    total_distance = 0
    for edge in best_path:
        total_distance += traffic_engine.edge_lengths.get(edge, 100.0)

    # Gather congestion info per edge
    edge_details = []
    for edge in best_path:
        length = traffic_engine.edge_lengths.get(edge, 100.0)
        cong_count = traffic_engine.aco.edge_congestion.get(edge, 0)
        speed_mps = max(1.0, 13.89 - cong_count * 1.5)
        speed_kmh = round(speed_mps * 3.6, 1)
        congestion = "High" if cong_count > 5 else "Medium" if cong_count > 2 else "Low"
        edge_details.append({
            "edge": edge,
            "length_m": round(length, 1),
            "speed_kmh": speed_kmh,
            "congestion": congestion,
            "vehicles_on_edge": cong_count
        })

    return {
        "origin": origin,
        "destination": destination,
        "path": best_path,
        "total_distance_m": round(total_distance, 1),
        "total_eta_seconds": round(best_cost, 1),
        "edges_count": len(best_path),
        "instructions": instructions,
        "edge_details": edge_details
    }


@app.get("/api/pheromones")
async def get_pheromones():
    """Returns current ACO pheromone levels on top edges."""
    return {"pheromones": traffic_engine.aco.get_pheromone_snapshot(20)}


@app.get("/api/convergence")
async def get_convergence():
    """Returns PSO convergence history (fitness score over time)."""
    return {
        "history": traffic_engine.pso.get_convergence(),
        "current_score": round(traffic_engine.pso.global_best_score, 3)
            if traffic_engine.pso.global_best_score != float('inf') else 0
    }

# Mount the static frontend
if os.path.exists(frontend_path):
    app.mount("/", StaticFiles(directory=frontend_path, html=True), name="frontend")
