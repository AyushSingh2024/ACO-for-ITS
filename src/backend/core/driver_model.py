import math

class DriverModel:
    def __init__(self, traffic_engine):
        self.te = traffic_engine

    def generate_instructions(self, path: list[str]) -> list[dict]:
        """
        Takes an ACO path (list of edge IDs) and generates turn-by-turn instructions.
        Uses edge_geoms and edge_lengths from traffic_engine.
        """
        if not path:
            return []

        instructions = []
        for i in range(len(path)):
            edge = path[i]
            length = self.te.edge_lengths.get(edge, 100.0)
            # Travel time via ACO heuristic
            eta = self.te.aco._travel_time(edge)

            action = "Go straight"
            if i > 0:
                prev_edge = path[i-1]
                action = self._determine_turn(prev_edge, edge)

            instructions.append({
                "step": i + 1,
                "road": f"Road {edge}",
                "action": action,
                "distance_m": round(length, 1),
                "eta_seconds": round(eta, 1)
            })
            
        # First instruction is always just driving straight on current road
        if instructions:
            instructions[0]["action"] = "Head straight"

        return instructions

    def _determine_turn(self, prev_edge: str, curr_edge: str) -> str:
        geom1 = self.te.edge_geoms.get(prev_edge)
        geom2 = self.te.edge_geoms.get(curr_edge)
        
        if not geom1 or not geom2:
            return "Go straight"

        # geom1 = ((x1_start, y1_start), (x1_end, y1_end))
        v1_x = geom1[1][0] - geom1[0][0]
        v1_y = geom1[1][1] - geom1[0][1]
        
        v2_x = geom2[1][0] - geom2[0][0]
        v2_y = geom2[1][1] - geom2[0][1]

        # Cross product (v1 x v2) 2D
        cross = (v1_x * v2_y) - (v1_y * v2_x)
        dot = (v1_x * v2_x) + (v1_y * v2_y)

        # Normalize to find angle
        len1 = math.sqrt(v1_x**2 + v1_y**2)
        len2 = math.sqrt(v2_x**2 + v2_y**2)

        if len1 == 0 or len2 == 0:
            return "Go straight"

        sin_theta = cross / (len1 * len2)
        cos_theta = dot / (len1 * len2)
        
        # Clamp to avoid domain errors
        cos_theta = max(-1.0, min(1.0, cos_theta))
        angle_rad = math.acos(cos_theta)
        angle_deg = math.degrees(angle_rad)

        if angle_deg < 25:
            return "Go straight"
        elif sin_theta > 0:
            return "Turn left"
        else:
            return "Turn right"
