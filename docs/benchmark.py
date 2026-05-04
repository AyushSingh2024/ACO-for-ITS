"""
Algorithm Comparison Benchmark
================================
Compares ACO vs Dijkstra vs Bellman-Ford vs A* on the real ACO-ITS road network.
Metrics: execution time, path cost (travel time), path length (edges).

Usage:
    From project root:
        python docs/benchmark.py

Output:
    docs/figures/  — PNG chart files
"""

import os
import sys
import time
import random
import heapq
import math
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Paths ────────────────────────────────────────────────────────────────────
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NET_PATH    = os.path.join(ROOT, 'data', 'simulation', 'network.net.xml')
ROUTES_PATH = os.path.join(ROOT, 'data', 'simulation', 'routes.rou.xml')
FIG_DIR     = os.path.join(ROOT, 'docs', 'figures')
os.makedirs(FIG_DIR, exist_ok=True)

# ── Styling ──────────────────────────────────────────────────────────────────
COLORS = {
    'ACO':          '#38bdf8',   # cyan-blue
    'Dijkstra':     '#34d399',   # green
    'Bellman-Ford': '#fbbf24',   # amber
    'A*':           '#c084fc',   # purple
}
plt.rcParams.update({
    'figure.facecolor': '#0a0f1e',
    'axes.facecolor':   '#0f172a',
    'axes.edgecolor':   '#1e293b',
    'axes.labelcolor':  '#94a3b8',
    'text.color':       '#f1f5f9',
    'xtick.color':      '#64748b',
    'ytick.color':      '#64748b',
    'grid.color':       '#1e293b',
    'grid.linestyle':   '--',
    'font.family':      'DejaVu Sans',
    'figure.dpi':       120,
})

N_OD_PAIRS   = 30    # number of origin-destination pairs to benchmark
ACO_N_ANTS   = 20    # ants per ACO run
ACO_RUNS     = 5     # full ACO cycles per OD pair (deposit+evaporate)
ALPHA        = 1.0
BETA         = 2.0
EVAPORATION  = 0.1
Q            = 100.0
INIT_PH      = 1.0
BASE_SPEED   = 13.89  # m/s ≈ 50 km/h
RANDOM_SEED  = 42

random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)


# ╔══════════════════════════════════════════════════════════╗
# ║  NETWORK LOADING                                         ║
# ╚══════════════════════════════════════════════════════════╝

def load_network(net_path):
    """Returns edge_lengths, edge_centroids, graph (adjacency)."""
    print("Loading network XML...")
    tree = ET.parse(net_path)
    root = tree.getroot()

    edge_lengths  = {}
    edge_centroids = {}   # edge_id -> (cx, cy)

    for edge in root.findall('edge'):
        eid = edge.get('id', '')
        if eid.startswith(':'):
            continue
        for lane in edge.findall('lane'):
            length = float(lane.get('length', 100.0))
            edge_lengths[eid] = length
            shape_str = lane.get('shape', '')
            coords = shape_str.split()
            pts = []
            for c in coords:
                parts = c.split(',')
                if len(parts) >= 2:
                    try:
                        pts.append((float(parts[0]), float(parts[1])))
                    except ValueError:
                        pass
            if pts:
                cx = sum(p[0] for p in pts) / len(pts)
                cy = sum(p[1] for p in pts) / len(pts)
                edge_centroids[eid] = (cx, cy)
            break  # one lane per edge

    print(f"  Loaded {len(edge_lengths)} edges.")
    return edge_lengths, edge_centroids


def load_routes(routes_path, edge_lengths):
    """Returns adjacency graph and pre-loaded routes."""
    print("Loading routes XML...")
    tree = ET.parse(routes_path)
    root = tree.getroot()

    graph  = defaultdict(set)   # edge -> set of next edges
    routes = []

    for vehicle in root.findall('vehicle'):
        route_el = vehicle.find('route')
        if route_el is None:
            continue
        edges = route_el.get('edges', '').split()
        # keep only edges that exist in our graph
        edges = [e for e in edges if e in edge_lengths]
        if len(edges) < 2:
            continue
        routes.append(edges)
        for i in range(len(edges) - 1):
            graph[edges[i]].add(edges[i + 1])

    print(f"  Built graph from {len(routes)} routes. Nodes: {len(graph)}")
    return graph, routes


def travel_time(eid, edge_lengths, congestion=None):
    length = edge_lengths.get(eid, 100.0)
    cong   = (congestion or {}).get(eid, 0)
    speed  = max(1.0, BASE_SPEED - cong * 1.5)
    return length / speed


# ╔══════════════════════════════════════════════════════════╗
# ║  ALGORITHM IMPLEMENTATIONS                               ║
# ╚══════════════════════════════════════════════════════════╝

# ── Dijkstra ──────────────────────────────────────────────
def dijkstra(graph, edge_lengths, origin, destination, congestion=None):
    dist  = {origin: 0.0}
    prev  = {}
    heap  = [(0.0, origin)]

    while heap:
        cost, u = heapq.heappop(heap)
        if u == destination:
            break
        if cost > dist.get(u, math.inf):
            continue
        for v in graph.get(u, []):
            w = travel_time(v, edge_lengths, congestion)
            nc = cost + w
            if nc < dist.get(v, math.inf):
                dist[v] = nc
                prev[v] = u
                heapq.heappush(heap, (nc, v))

    if destination not in dist:
        return None, math.inf

    path = []
    cur  = destination
    while cur in prev:
        path.append(cur)
        cur = prev[cur]
    path.append(origin)
    path.reverse()
    return path, dist[destination]


# ── Bellman-Ford ─────────────────────────────────────────
def bellman_ford(graph, edge_lengths, origin, destination, congestion=None):
    """
    Standard Bellman-Ford on the edge-adjacency graph.
    Since the graph can be large, we run a BFS-scoped version:
    first collect reachable nodes from origin, then relax only those.
    """
    # BFS to find reachable nodes (limits work on sparse graph)
    from collections import deque
    reachable = set()
    q = deque([origin])
    reachable.add(origin)
    while q:
        u = q.popleft()
        for v in graph.get(u, []):
            if v not in reachable:
                reachable.add(v)
                q.append(v)
            if destination in reachable and len(reachable) > 5000:
                break  # cap size for performance

    if destination not in reachable:
        return None, math.inf

    nodes = list(reachable)
    dist  = {n: math.inf for n in nodes}
    prev  = {}
    dist[origin] = 0.0

    # Build edge list within reachable set
    edges_list = []
    for u in nodes:
        for v in graph.get(u, []):
            if v in reachable:
                edges_list.append((u, v, travel_time(v, edge_lengths, congestion)))

    # Relax |V|-1 times (early exit if no change)
    for _ in range(len(nodes) - 1):
        updated = False
        for u, v, w in edges_list:
            if dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                prev[v]  = u
                updated  = True
        if not updated:
            break

    if dist[destination] == math.inf:
        return None, math.inf

    path = []
    cur  = destination
    while cur in prev:
        path.append(cur)
        cur = prev[cur]
    path.append(origin)
    path.reverse()
    return path, dist[destination]


# ── A* ───────────────────────────────────────────────────
def astar(graph, edge_lengths, edge_centroids, origin, destination, congestion=None):
    """A* using Euclidean distance (in network coordinates) as heuristic."""
    def heuristic(u, goal):
        if u not in edge_centroids or goal not in edge_centroids:
            return 0.0
        ux, uy = edge_centroids[u]
        gx, gy = edge_centroids[goal]
        return math.sqrt((ux - gx)**2 + (uy - gy)**2) / BASE_SPEED

    g_cost = {origin: 0.0}
    f_cost = {origin: heuristic(origin, destination)}
    prev   = {}
    heap   = [(f_cost[origin], origin)]

    while heap:
        _, u = heapq.heappop(heap)
        if u == destination:
            break
        for v in graph.get(u, []):
            w  = travel_time(v, edge_lengths, congestion)
            ng = g_cost.get(u, math.inf) + w
            if ng < g_cost.get(v, math.inf):
                g_cost[v] = ng
                f_cost[v] = ng + heuristic(v, destination)
                prev[v]   = u
                heapq.heappush(heap, (f_cost[v], v))

    if destination not in g_cost:
        return None, math.inf

    path = []
    cur  = destination
    while cur in prev:
        path.append(cur)
        cur = prev[cur]
    path.append(origin)
    path.reverse()
    return path, g_cost[destination]


# ── ACO ──────────────────────────────────────────────────
def aco(graph, edge_lengths, edge_centroids, origin, destination,
        n_ants=ACO_N_ANTS, n_runs=ACO_RUNS, congestion=None):
    pheromone = defaultdict(lambda: INIT_PH)

    def _tt(eid):
        return travel_time(eid, edge_lengths, congestion)

    def heuristic(u, goal):
        if u not in edge_centroids or goal not in edge_centroids:
            return 0.0
        ux, uy = edge_centroids[u]
        gx, gy = edge_centroids[goal]
        return math.sqrt((ux - gx)**2 + (uy - gy)**2) / BASE_SPEED

    def ant_walk():
        path    = [origin]
        current = origin
        visited = {origin}
        for _ in range(200):
            if current == destination:
                break
            neighbors = [n for n in graph.get(current, []) if n not in visited]
            if not neighbors:
                break
            
            scores = []
            for n in neighbors:
                tau = pheromone[n] ** ALPHA
                est_rem = _tt(n) + heuristic(n, destination)
                eta = (1.0 / max(0.1, est_rem)) ** BETA
                scores.append(tau * eta)
            total = sum(scores)
            if total == 0:
                nxt = random.choice(neighbors)
            else:
                probs = [s / total for s in scores]
                nxt   = random.choices(neighbors, weights=probs, k=1)[0]
            path.append(nxt)
            visited.add(nxt)
            current = nxt
        return path

    best_path = None
    best_cost = math.inf

    for _ in range(n_runs):
        # evaporate
        for k in list(pheromone.keys()):
            pheromone[k] *= (1 - EVAPORATION)
            if pheromone[k] < 0.01:
                pheromone[k] = 0.01

        for _ in range(n_ants):
            path = ant_walk()
            if path and path[-1] == destination:
                cost = sum(_tt(e) for e in path)
                if cost < best_cost:
                    best_cost = cost
                    best_path = path
                # deposit
                if cost > 0:
                    deposit = Q / cost
                    for e in path:
                        pheromone[e] += deposit

    return best_path, best_cost


# ╔══════════════════════════════════════════════════════════╗
# ║  BENCHMARK RUNNER                                        ║
# ╚══════════════════════════════════════════════════════════╝

def run_benchmark(graph, edge_lengths, edge_centroids, od_pairs):
    algos = ['Dijkstra', 'Bellman-Ford', 'A*', 'ACO']
    results = {a: {'times': [], 'costs': [], 'lengths': [], 'found': 0} for a in algos}

    print(f"\nBenchmarking {len(od_pairs)} OD pairs...")
    for idx, (origin, dest) in enumerate(od_pairs):
        print(f"  [{idx+1:02d}/{len(od_pairs)}] {origin[:16]} -> {dest[:16]}", end='', flush=True)

        # Dijkstra
        t0 = time.perf_counter()
        path_d, cost_d = dijkstra(graph, edge_lengths, origin, dest)
        t_d = (time.perf_counter() - t0) * 1000
        if path_d:
            results['Dijkstra']['times'].append(t_d)
            results['Dijkstra']['costs'].append(cost_d)
            results['Dijkstra']['lengths'].append(len(path_d))
            results['Dijkstra']['found'] += 1

        # Bellman-Ford
        t0 = time.perf_counter()
        path_b, cost_b = bellman_ford(graph, edge_lengths, origin, dest)
        t_b = (time.perf_counter() - t0) * 1000
        if path_b:
            results['Bellman-Ford']['times'].append(t_b)
            results['Bellman-Ford']['costs'].append(cost_b)
            results['Bellman-Ford']['lengths'].append(len(path_b))
            results['Bellman-Ford']['found'] += 1

        # A*
        t0 = time.perf_counter()
        path_a, cost_a = astar(graph, edge_lengths, edge_centroids, origin, dest)
        t_a = (time.perf_counter() - t0) * 1000
        if path_a:
            results['A*']['times'].append(t_a)
            results['A*']['costs'].append(cost_a)
            results['A*']['lengths'].append(len(path_a))
            results['A*']['found'] += 1

        # ACO
        t0 = time.perf_counter()
        path_aco, cost_aco = aco(graph, edge_lengths, edge_centroids, origin, dest)
        t_aco = (time.perf_counter() - t0) * 1000
        if path_aco:
            results['ACO']['times'].append(t_aco)
            results['ACO']['costs'].append(cost_aco)
            results['ACO']['lengths'].append(len(path_aco))
            results['ACO']['found'] += 1

        # Optimality gap (ACO vs Dijkstra)
        print(f"  OK")

    return results


# ╔══════════════════════════════════════════════════════════╗
# ║  CHARTS                                                  ║
# ╚══════════════════════════════════════════════════════════╝

def save(fig, name):
    path = os.path.join(FIG_DIR, name)
    fig.savefig(path, bbox_inches='tight', facecolor=fig.get_facecolor())
    print(f"  Saved -> {path}")
    plt.close(fig)


def fmt_ms(val):
    return f"{val:.1f} ms"


def chart_bar(results, key, title, ylabel, filename, aggfn=np.mean):
    algos = list(COLORS.keys())
    vals  = [aggfn(results[a][key]) if results[a][key] else 0 for a in algos]
    errs  = [np.std(results[a][key]) if results[a][key] else 0 for a in algos]
    cols  = [COLORS[a] for a in algos]

    fig, ax = plt.subplots(figsize=(9, 5))
    bars = ax.bar(algos, vals, yerr=errs, color=cols,
                  capsize=5, width=0.55,
                  error_kw={'ecolor': '#475569', 'elinewidth': 1.5})

    for bar, val, err in zip(bars, vals, errs):
        ax.text(bar.get_x() + bar.get_width() / 2,
                bar.get_height() + err + max(vals) * 0.01,
                f"{val:.2f}", ha='center', va='bottom',
                fontsize=10, fontweight='bold', color='#f1f5f9')

    ax.set_title(title, fontsize=14, fontweight='bold', pad=14, color='#f1f5f9')
    ax.set_ylabel(ylabel, fontsize=11)
    ax.yaxis.grid(True, alpha=0.4)
    ax.set_axisbelow(True)
    fig.tight_layout()
    save(fig, filename)


def chart_box(results, key, title, ylabel, filename):
    algos = list(COLORS.keys())
    data  = [results[a][key] for a in algos]
    cols  = [COLORS[a] for a in algos]

    fig, ax = plt.subplots(figsize=(9, 5))
    bp = ax.boxplot(data, patch_artist=True, notch=False,
                    medianprops={'color': '#f1f5f9', 'linewidth': 2},
                    whiskerprops={'color': '#475569'},
                    capprops={'color': '#475569'},
                    flierprops={'marker': 'o', 'markersize': 4,
                                'markerfacecolor': '#64748b', 'linestyle': 'none'})
    for patch, col in zip(bp['boxes'], cols):
        patch.set_facecolor(col)
        patch.set_alpha(0.75)

    ax.set_xticks(range(1, len(algos) + 1))
    ax.set_xticklabels(algos, fontsize=11)
    ax.set_title(title, fontsize=14, fontweight='bold', pad=14, color='#f1f5f9')
    ax.set_ylabel(ylabel, fontsize=11)
    ax.yaxis.grid(True, alpha=0.4)
    ax.set_axisbelow(True)
    fig.tight_layout()
    save(fig, filename)


def chart_scatter_time_vs_cost(results, filename):
    fig, ax = plt.subplots(figsize=(9, 6))
    for algo, col in COLORS.items():
        if not results[algo]['times']:
            continue
        ax.scatter(results[algo]['times'], results[algo]['costs'],
                   color=col, alpha=0.75, s=55, label=algo, edgecolors='none')

    ax.set_xlabel('Execution Time (ms)', fontsize=11)
    ax.set_ylabel('Path Cost (travel time, s)', fontsize=11)
    ax.set_title('Execution Time vs Path Cost', fontsize=14,
                 fontweight='bold', pad=14, color='#f1f5f9')
    ax.legend(loc='upper left', fontsize=10,
              framealpha=0.25, facecolor='#0f172a', edgecolor='#1e293b')
    ax.grid(True, alpha=0.4)
    fig.tight_layout()
    save(fig, filename)


def chart_optimality_gap(results, filename):
    """Show how much more time ACO's paths take vs Dijkstra (optimal)."""
    dijk_costs = results['Dijkstra']['costs']
    aco_costs  = results['ACO']['costs']
    n = min(len(dijk_costs), len(aco_costs))
    if n == 0:
        print("  Skipping optimality gap chart (no data).")
        return

    gaps = [((aco_costs[i] - dijk_costs[i]) / max(dijk_costs[i], 1)) * 100
            for i in range(n)]

    fig, ax = plt.subplots(figsize=(10, 5))
    xs = range(1, n + 1)
    ax.bar(xs, gaps, color='#f87171', alpha=0.72, width=0.7)
    ax.axhline(np.mean(gaps), color='#fbbf24', linewidth=1.8,
               linestyle='--', label=f'Mean gap: {np.mean(gaps):.1f}%')
    ax.axhline(0, color='#34d399', linewidth=1.2, linestyle='-', label='Optimal (Dijkstra)')

    ax.set_xlabel('OD Pair Index', fontsize=11)
    ax.set_ylabel('Cost Overhead vs Dijkstra (%)', fontsize=11)
    ax.set_title('ACO Optimality Gap vs Dijkstra', fontsize=14,
                 fontweight='bold', pad=14, color='#f1f5f9')
    ax.legend(fontsize=10, framealpha=0.25,
              facecolor='#0f172a', edgecolor='#1e293b')
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_axisbelow(True)
    fig.tight_layout()
    save(fig, filename)


def chart_path_length_comparison(results, filename):
    algos = list(COLORS.keys())
    means = [np.mean(results[a]['lengths']) if results[a]['lengths'] else 0 for a in algos]
    stds  = [np.std(results[a]['lengths'])  if results[a]['lengths'] else 0 for a in algos]
    cols  = [COLORS[a] for a in algos]

    fig, ax = plt.subplots(figsize=(9, 5))
    bars = ax.bar(algos, means, yerr=stds, color=cols,
                  capsize=5, width=0.55,
                  error_kw={'ecolor': '#475569', 'elinewidth': 1.5})
    for bar, val in zip(bars, means):
        ax.text(bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 0.3,
                f"{val:.1f}", ha='center', va='bottom',
                fontsize=10, fontweight='bold', color='#f1f5f9')
    ax.set_title('Average Path Length (Edges)', fontsize=14,
                 fontweight='bold', pad=14, color='#f1f5f9')
    ax.set_ylabel('Number of Edges', fontsize=11)
    ax.yaxis.grid(True, alpha=0.4)
    ax.set_axisbelow(True)
    fig.tight_layout()
    save(fig, filename)


def chart_success_rate(results, n_pairs, filename):
    algos = list(COLORS.keys())
    rates = [results[a]['found'] / n_pairs * 100 for a in algos]
    cols  = [COLORS[a] for a in algos]

    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(algos, rates, color=cols, width=0.5)
    for bar, val in zip(bars, rates):
        ax.text(bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 0.5,
                f"{val:.0f}%", ha='center', va='bottom',
                fontsize=11, fontweight='bold', color='#f1f5f9')
    ax.set_ylim(0, 115)
    ax.set_title('Path Found Rate', fontsize=14,
                 fontweight='bold', pad=14, color='#f1f5f9')
    ax.set_ylabel('% of OD Pairs Solved', fontsize=11)
    ax.yaxis.grid(True, alpha=0.4)
    ax.set_axisbelow(True)
    fig.tight_layout()
    save(fig, filename)


def chart_summary_table(results, filename):
    """Render a styled summary table as an image."""
    algos = list(COLORS.keys())
    rows  = []
    for a in algos:
        r = results[a]
        rows.append([
            a,
            f"{np.mean(r['times']):.2f} ms"     if r['times']   else '—',
            f"{np.std(r['times']):.2f} ms"      if r['times']   else '—',
            f"{np.mean(r['costs']):.1f} s"      if r['costs']   else '—',
            f"{np.mean(r['lengths']):.1f}"      if r['lengths'] else '—',
            f"{r['found']}/{len(r['times']) or '?'}",
        ])

    col_labels = ['Algorithm', 'Mean Time', 'Std Time',
                  'Mean Path Cost', 'Mean Path Len', 'Paths Found']

    fig, ax = plt.subplots(figsize=(12, 3))
    ax.axis('off')
    tbl = ax.table(cellText=rows, colLabels=col_labels,
                   loc='center', cellLoc='center')
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(11)
    tbl.scale(1, 2.2)

    for (row, col), cell in tbl.get_celld().items():
        cell.set_edgecolor('#1e293b')
        if row == 0:
            cell.set_facecolor('#1e3a5f')
            cell.set_text_props(color='#f1f5f9', fontweight='bold')
        else:
            algo = algos[row - 1]
            cell.set_facecolor('#0f172a' if row % 2 == 0 else '#111827')
            if col == 0:
                cell.set_facecolor(COLORS[algo])
                cell.set_text_props(color='#020817', fontweight='bold')
            else:
                cell.set_text_props(color='#cbd5e1')

    ax.set_title('Algorithm Comparison — Summary', fontsize=13,
                 fontweight='bold', pad=20, color='#f1f5f9')
    fig.tight_layout()
    save(fig, filename)


# ╔══════════════════════════════════════════════════════════╗
# ║  MAIN                                                    ║
# ╚══════════════════════════════════════════════════════════╝

def main():
    # ── Load data ────────────────────────────────────────────
    edge_lengths, edge_centroids = load_network(NET_PATH)
    graph, routes = load_routes(ROUTES_PATH, edge_lengths)

    if not routes:
        print("ERROR: No routes loaded. Check ROUTES_PATH.")
        return

    # ── Select OD pairs from actual routes ───────────────────
    # Pick pairs from real vehicle routes so they are guaranteed reachable
    random.shuffle(routes)
    od_pairs = []
    for r in routes:
        if len(r) >= 5:   # only use routes with enough edges
            origin = r[0]
            dest   = r[-1]
            if origin != dest and origin in graph and dest in edge_lengths:
                od_pairs.append((origin, dest))
        if len(od_pairs) >= N_OD_PAIRS:
            break

    if not od_pairs:
        print("ERROR: Could not extract OD pairs from routes.")
        return

    print(f"\nSelected {len(od_pairs)} OD pairs from real vehicle routes.")
    print(f"Graph has {len(graph)} nodes (edges with outgoing connections).")

    # ── Run benchmark ─────────────────────────────────────────
    results = run_benchmark(graph, edge_lengths, edge_centroids, od_pairs)

    # ── Print summary ─────────────────────────────────────────
    print("\n" + "="*62)
    print(f"{'Algorithm':<15} {'Mean Time':>12} {'Mean Cost':>12} {'Path Len':>10} {'Found':>8}")
    print("-"*62)
    for a in COLORS:
        r = results[a]
        mt  = f"{np.mean(r['times']):.2f} ms"    if r['times']   else '—'
        mc  = f"{np.mean(r['costs']):.1f} s"     if r['costs']   else '—'
        ml  = f"{np.mean(r['lengths']):.1f}"     if r['lengths'] else '—'
        fnd = f"{r['found']}/{len(od_pairs)}"
        print(f"{a:<15} {mt:>12} {mc:>12} {ml:>10} {fnd:>8}")
    print("="*62)

    # ── Generate charts ───────────────────────────────────────
    print("\nGenerating charts...")
    chart_bar(results, 'times',
              'Average Execution Time per Algorithm',
              'Time (ms)', '01_avg_execution_time.png')

    chart_box(results, 'times',
              'Execution Time Distribution',
              'Time (ms)', '02_time_distribution.png')

    chart_bar(results, 'costs',
              'Average Path Cost (Estimated Travel Time)',
              'Travel Time (s)', '03_avg_path_cost.png')

    chart_box(results, 'costs',
              'Path Cost Distribution',
              'Travel Time (s)', '04_cost_distribution.png')

    chart_scatter_time_vs_cost(results, '05_time_vs_cost_scatter.png')

    chart_optimality_gap(results, '06_aco_optimality_gap.png')

    chart_path_length_comparison(results, '07_path_length.png')

    chart_success_rate(results, len(od_pairs), '08_success_rate.png')

    chart_summary_table(results, '09_summary_table.png')

    print(f"\nAll charts saved to: {FIG_DIR}")


if __name__ == '__main__':
    main()
