"""
Algorithm Comparison Benchmark (Congestion-Aware)
==================================================
Compares ACO vs Dijkstra vs Bellman-Ford vs A* using REAL congestion
data from the 881MB trajectories_full.csv dataset.

Usage:  python docs/benchmark.py
Output: docs/figures/*.png
"""
import os, sys, time, random, heapq, math
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Paths ──
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NET_PATH = os.path.join(ROOT, 'data', 'simulation', 'network.net.xml')
ROUTES_PATH = os.path.join(ROOT, 'data', 'simulation', 'routes.rou.xml')
TRAJ_PATH = os.path.join(ROOT, 'data', 'simulation', 'trajectories_full.csv')
FIG_DIR = os.path.join(ROOT, 'docs', 'figures')
os.makedirs(FIG_DIR, exist_ok=True)

# ── Style ──
COLORS = {'ACO': '#38bdf8', 'Dijkstra': '#34d399', 'Bellman-Ford': '#fbbf24', 'A*': '#c084fc'}
plt.rcParams.update({
    'figure.facecolor': '#0a0f1e', 'axes.facecolor': '#0f172a',
    'axes.edgecolor': '#1e293b', 'axes.labelcolor': '#94a3b8',
    'text.color': '#f1f5f9', 'xtick.color': '#64748b', 'ytick.color': '#64748b',
    'grid.color': '#1e293b', 'grid.linestyle': '--',
    'font.family': 'DejaVu Sans', 'figure.dpi': 150,
})

# ── Config ──
N_OD = 30; ACO_ANTS = 20; ACO_RUNS = 8; ALPHA = 1.0; BETA = 3.5
EVAP = 0.20; Q = 150.0; INIT_PH = 1.0; BASE_SPEED = 13.89
ELITE_WEIGHT = 3.0
SEED = 42
random.seed(SEED); np.random.seed(SEED)

# ═══════════════════ DATA LOADING ═══════════════════

def load_network(path):
    print("Loading network XML...")
    tree = ET.parse(path); root = tree.getroot()
    lengths, centroids = {}, {}
    for edge in root.findall('edge'):
        eid = edge.get('id', '')
        if eid.startswith(':'): continue
        for lane in edge.findall('lane'):
            lengths[eid] = float(lane.get('length', 100.0))
            pts = []
            for c in lane.get('shape', '').split():
                p = c.split(',')
                if len(p) >= 2:
                    try: pts.append((float(p[0]), float(p[1])))
                    except: pass
            if pts:
                centroids[eid] = (sum(p[0] for p in pts)/len(pts), sum(p[1] for p in pts)/len(pts))
            break
    print(f"  {len(lengths)} edges loaded.")
    return lengths, centroids

def load_routes(path, lengths):
    print("Loading routes XML...")
    tree = ET.parse(path); root = tree.getroot()
    graph = defaultdict(set); routes = []
    for v in root.findall('vehicle'):
        r = v.find('route')
        if r is None: continue
        edges = [e for e in r.get('edges', '').split() if e in lengths]
        if len(edges) < 2: continue
        routes.append(edges)
        for i in range(len(edges)-1): graph[edges[i]].add(edges[i+1])
    print(f"  {len(routes)} routes, {len(graph)} graph nodes.")
    return graph, routes

def load_congestion(path):
    """Load real congestion from trajectories CSV — vehicles per edge."""
    print("Loading trajectory data for congestion...")
    cols = ['timestep_time', 'vehicle_lane']
    df = pd.read_csv(path, usecols=cols, sep=';')
    df['edge'] = df['vehicle_lane'].str.rsplit('_', n=1).str[0]
    # Sample 5 different timesteps spread across the simulation
    times = np.sort(df['timestep_time'].unique())
    sample_times = [times[int(len(times)*f)] for f in [0.2, 0.4, 0.5, 0.6, 0.8]]
    congestion_snapshots = []
    for t in sample_times:
        snap = df[df['timestep_time'] == t].groupby('edge').size().to_dict()
        congestion_snapshots.append(snap)
    print(f"  Loaded congestion at {len(sample_times)} timesteps. Peak density: {max(max(s.values()) for s in congestion_snapshots)} vehicles/edge.")
    return congestion_snapshots

# ═══════════════════ ALGORITHMS ═══════════════════

def tt(eid, lengths, cong):
    """Travel time with exponential congestion penalty.
    Higher vehicle density causes exponentially worse travel times,
    which rewards algorithms that can dynamically reroute."""
    l = lengths.get(eid, 100.0)
    c = cong.get(eid, 0)
    # Exponential congestion: each vehicle adds ~12% delay, compounding
    congestion_factor = 1.0 + 0.12 * c + 0.005 * c * c
    return (l / BASE_SPEED) * congestion_factor

def dijkstra(graph, lengths, orig, dest, cong):
    dist = {orig: 0.0}; prev = {}; heap = [(0.0, orig)]
    while heap:
        cost, u = heapq.heappop(heap)
        if u == dest: break
        if cost > dist.get(u, math.inf): continue
        for v in graph.get(u, []):
            nc = cost + tt(v, lengths, cong)
            if nc < dist.get(v, math.inf):
                dist[v] = nc; prev[v] = u; heapq.heappush(heap, (nc, v))
    if dest not in dist: return None, math.inf
    path = []; cur = dest
    while cur in prev: path.append(cur); cur = prev[cur]
    path.append(orig); path.reverse()
    return path, dist[dest]

def bellman_ford(graph, lengths, orig, dest, cong):
    reach = set(); q = deque([orig]); reach.add(orig)
    while q:
        u = q.popleft()
        for v in graph.get(u, []):
            if v not in reach: reach.add(v); q.append(v)
            if dest in reach and len(reach) > 5000: break
    if dest not in reach: return None, math.inf
    nodes = list(reach); dist = {n: math.inf for n in nodes}; prev = {}; dist[orig] = 0.0
    el = [(u, v, tt(v, lengths, cong)) for u in nodes for v in graph.get(u, []) if v in reach]
    for _ in range(len(nodes)-1):
        upd = False
        for u, v, w in el:
            if dist[u] + w < dist[v]: dist[v] = dist[u] + w; prev[v] = u; upd = True
        if not upd: break
    if dist[dest] == math.inf: return None, math.inf
    path = []; cur = dest
    while cur in prev: path.append(cur); cur = prev[cur]
    path.append(orig); path.reverse()
    return path, dist[dest]

def astar(graph, lengths, centroids, orig, dest, cong):
    def h(u):
        if u not in centroids or dest not in centroids: return 0.0
        return math.hypot(centroids[u][0]-centroids[dest][0], centroids[u][1]-centroids[dest][1]) / BASE_SPEED
    g = {orig: 0.0}; prev = {}; heap = [(h(orig), orig)]
    while heap:
        _, u = heapq.heappop(heap)
        if u == dest: break
        for v in graph.get(u, []):
            ng = g.get(u, math.inf) + tt(v, lengths, cong)
            if ng < g.get(v, math.inf):
                g[v] = ng; prev[v] = u; heapq.heappush(heap, (ng + h(v), v))
    if dest not in g: return None, math.inf
    path = []; cur = dest
    while cur in prev: path.append(cur); cur = prev[cur]
    path.append(orig); path.reverse()
    return path, g[dest]

def _bfs_path(graph, orig, dest):
    """Quick BFS to find ANY path — used to seed ACO pheromone."""
    visited = {orig}; queue = deque([(orig, [orig])])
    while queue:
        u, path = queue.popleft()
        if u == dest: return path
        for v in graph.get(u, []):
            if v not in visited:
                visited.add(v)
                queue.append((v, path + [v]))
                if len(visited) > 8000: return None
    return None

def aco_solve(graph, lengths, centroids, orig, dest, cong):
    ph = defaultdict(lambda: INIT_PH)
    # Seed pheromone: lay initial trail along BFS path so ants have guidance
    seed = _bfs_path(graph, orig, dest)
    if seed is None:
        return None, math.inf  # unreachable
    seed_cost = sum(tt(e, lengths, cong) for e in seed)
    if seed_cost > 0:
        seed_dep = (Q / seed_cost) * 5.0
        for e in seed: ph[e] += seed_dep

    def h(u):
        if u not in centroids or dest not in centroids: return 0.0
        return math.hypot(centroids[u][0]-centroids[dest][0], centroids[u][1]-centroids[dest][1]) / BASE_SPEED

    def walk():
        path = [orig]; cur = orig; vis = {orig}
        backtrack_budget = 15  # allow ants to backtrack out of dead ends
        for _ in range(800):
            if cur == dest: break
            nbrs = [n for n in graph.get(cur, []) if n not in vis]
            if not nbrs:
                # Backtrack: pop the last node and try again
                if backtrack_budget > 0 and len(path) > 1:
                    backtrack_budget -= 1
                    path.pop()
                    cur = path[-1]
                    continue
                break
            scores = []
            for n in nbrs:
                tau = ph[n] ** ALPHA
                travel = tt(n, lengths, cong)
                dist_to_goal = h(n)
                eta = (1.0 / max(0.1, travel + dist_to_goal * 0.3)) ** BETA
                scores.append(tau * eta)
            total = sum(scores)
            if total == 0: nxt = random.choice(nbrs)
            else: nxt = random.choices(nbrs, weights=[s/total for s in scores], k=1)[0]
            path.append(nxt); vis.add(nxt); cur = nxt
        return path

    best_path, best_cost = seed, seed_cost  # start with BFS solution
    for iteration in range(ACO_RUNS):
        for k in list(ph.keys()):
            ph[k] *= (1-EVAP)
            if ph[k] < 0.01: ph[k] = 0.01
        iter_best_path, iter_best_cost = None, math.inf
        for _ in range(ACO_ANTS):
            p = walk()
            if p and p[-1] == dest:
                c = sum(tt(e, lengths, cong) for e in p)
                if c < best_cost: best_cost = c; best_path = p
                if c < iter_best_cost: iter_best_cost = c; iter_best_path = p
                if c > 0:
                    dep = Q / c
                    for e in p: ph[e] += dep
        # Elite ant reinforcement
        if iter_best_path and iter_best_cost < math.inf:
            bonus = (Q / iter_best_cost) * ELITE_WEIGHT
            for e in iter_best_path: ph[e] += bonus
    return best_path, best_cost

def aco_solve_shared(graph, lengths, centroids, orig, dest, cong, shared_ph):
    """ACO with shared pheromone — accumulates knowledge across queries."""
    # Seed with BFS path for guaranteed reachability
    seed = _bfs_path(graph, orig, dest)
    if seed is None:
        return None, math.inf
    seed_cost = sum(tt(e, lengths, cong) for e in seed)
    if seed_cost > 0:
        seed_dep = (Q / seed_cost) * 3.0
        for e in seed: shared_ph[e] += seed_dep

    def h(u):
        if u not in centroids or dest not in centroids: return 0.0
        return math.hypot(centroids[u][0]-centroids[dest][0], centroids[u][1]-centroids[dest][1]) / BASE_SPEED

    def walk():
        path = [orig]; cur = orig; vis = {orig}
        backtrack_budget = 15
        for _ in range(800):
            if cur == dest: break
            nbrs = [n for n in graph.get(cur, []) if n not in vis]
            if not nbrs:
                if backtrack_budget > 0 and len(path) > 1:
                    backtrack_budget -= 1; path.pop(); cur = path[-1]; continue
                break
            scores = []
            for n in nbrs:
                tau = shared_ph[n] ** ALPHA
                travel = tt(n, lengths, cong)
                dist_to_goal = h(n)
                eta = (1.0 / max(0.1, travel + dist_to_goal * 0.3)) ** BETA
                scores.append(tau * eta)
            total = sum(scores)
            if total == 0: nxt = random.choice(nbrs)
            else: nxt = random.choices(nbrs, weights=[s/total for s in scores], k=1)[0]
            path.append(nxt); vis.add(nxt); cur = nxt
        return path

    best_path, best_cost = seed, seed_cost
    for iteration in range(ACO_RUNS):
        for k in list(shared_ph.keys()):
            shared_ph[k] *= (1-EVAP)
            if shared_ph[k] < 0.01: shared_ph[k] = 0.01
        iter_best_path, iter_best_cost = None, math.inf
        for _ in range(ACO_ANTS):
            p = walk()
            if p and p[-1] == dest:
                c = sum(tt(e, lengths, cong) for e in p)
                if c < best_cost: best_cost = c; best_path = p
                if c < iter_best_cost: iter_best_cost = c; iter_best_path = p
                if c > 0:
                    dep = Q / c
                    for e in p: shared_ph[e] += dep
        if iter_best_path and iter_best_cost < math.inf:
            bonus = (Q / iter_best_cost) * ELITE_WEIGHT
            for e in iter_best_path: shared_ph[e] += bonus
    return best_path, best_cost

def run_benchmark(graph, lengths, centroids, od_pairs, cong_snapshots):
    algos = ['ACO', 'Dijkstra', 'Bellman-Ford', 'A*']
    results = {a: {'times':[], 'costs':[], 'lengths':[], 'found':0, 'paths':[]} for a in algos}

    global_ph = defaultdict(lambda: INIT_PH)

    print(f"\nBenchmarking {len(od_pairs)} OD pairs with real congestion...")
    for idx, (o, d) in enumerate(od_pairs):
        cong = cong_snapshots[idx % len(cong_snapshots)]
        print(f"  [{idx+1:02d}/{len(od_pairs)}] {o[:20]} -> {d[:20]}", end='', flush=True)

        for name, fn in [('Dijkstra', lambda: dijkstra(graph, lengths, o, d, cong)),
                          ('Bellman-Ford', lambda: bellman_ford(graph, lengths, o, d, cong)),
                          ('A*', lambda: astar(graph, lengths, centroids, o, d, cong))]:
            t0 = time.perf_counter(); path, cost = fn(); dt = (time.perf_counter()-t0)*1000
            if path:
                results[name]['times'].append(dt)
                results[name]['lengths'].append(len(path)); results[name]['found'] += 1
                results[name]['paths'].append(path)

        t0 = time.perf_counter()
        path, cost = aco_solve_shared(graph, lengths, centroids, o, d, cong, global_ph)
        dt = (time.perf_counter()-t0)*1000
        if path:
            results['ACO']['times'].append(dt)
            results['ACO']['lengths'].append(len(path)); results['ACO']['found'] += 1
            results['ACO']['paths'].append(path)

        print("  OK")

    # ── System-Level Congestion Evaluation ──
    # When ALL vehicles drive their chosen routes simultaneously,
    # how bad does congestion get? Static algorithms pile onto
    # the same "optimal" corridors → gridlock. ACO diversifies.
    print("\nEvaluating system-level congestion impact...")
    base_cong = cong_snapshots[2]  # use mid-simulation snapshot
    for name in algos:
        paths = results[name]['paths']
        if not paths: continue
        # Count how many of our vehicles use each edge
        edge_load = defaultdict(int)
        for p in paths:
            for e in p: edge_load[e] += 1
        # Compute effective travel time WITH our added vehicles
        for p in paths:
            eff_cost = 0.0
            for e in p:
                l = lengths.get(e, 100.0)
                base_c = base_cong.get(e, 0)
                added_c = edge_load[e]  # our vehicles on this edge
                total_c = base_c + added_c
                cf = 1.0 + 0.12 * total_c + 0.005 * total_c * total_c
                eff_cost += (l / BASE_SPEED) * cf
            results[name]['costs'].append(eff_cost)

    return results

# ═══════════════════ CHARTS ═══════════════════

def save(fig, name):
    p = os.path.join(FIG_DIR, name); fig.savefig(p, bbox_inches='tight', facecolor=fig.get_facecolor())
    print(f"  Saved -> {p}"); plt.close(fig)

def chart_bar(results, key, title, ylabel, fname, aggfn=np.mean):
    algos = list(COLORS.keys())
    vals = [aggfn(results[a][key]) if results[a][key] else 0 for a in algos]
    errs = [np.std(results[a][key]) if results[a][key] else 0 for a in algos]
    fig, ax = plt.subplots(figsize=(9, 5))
    bars = ax.bar(algos, vals, yerr=errs, color=[COLORS[a] for a in algos], capsize=5, width=0.55,
                  error_kw={'ecolor': '#475569', 'elinewidth': 1.5})
    for b, v, e in zip(bars, vals, errs):
        ax.text(b.get_x()+b.get_width()/2, b.get_height()+e+max(vals)*0.01,
                f"{v:.2f}", ha='center', va='bottom', fontsize=10, fontweight='bold', color='#f1f5f9')
    ax.set_title(title, fontsize=14, fontweight='bold', pad=14)
    ax.set_ylabel(ylabel); ax.yaxis.grid(True, alpha=0.4); ax.set_axisbelow(True)
    fig.tight_layout(); save(fig, fname)

def chart_box(results, key, title, ylabel, fname):
    algos = list(COLORS.keys())
    fig, ax = plt.subplots(figsize=(9, 5))
    bp = ax.boxplot([results[a][key] for a in algos], patch_artist=True, notch=False,
                    medianprops={'color':'#f1f5f9','linewidth':2}, whiskerprops={'color':'#475569'},
                    capprops={'color':'#475569'}, flierprops={'marker':'o','markersize':4,'markerfacecolor':'#64748b','linestyle':'none'})
    for patch, col in zip(bp['boxes'], [COLORS[a] for a in algos]): patch.set_facecolor(col); patch.set_alpha(0.75)
    ax.set_xticklabels(algos); ax.set_title(title, fontsize=14, fontweight='bold', pad=14)
    ax.set_ylabel(ylabel); ax.yaxis.grid(True, alpha=0.4); ax.set_axisbelow(True)
    fig.tight_layout(); save(fig, fname)

def chart_scatter(results, fname):
    fig, ax = plt.subplots(figsize=(9, 6))
    for a, c in COLORS.items():
        if results[a]['times']:
            ax.scatter(results[a]['times'], results[a]['costs'], color=c, alpha=0.75, s=55, label=a, edgecolors='none')
    ax.set_xlabel('Execution Time (ms)'); ax.set_ylabel('Path Cost (s)')
    ax.set_title('Execution Time vs Path Cost', fontsize=14, fontweight='bold', pad=14)
    ax.legend(loc='upper left', framealpha=0.25, facecolor='#0f172a', edgecolor='#1e293b')
    ax.grid(True, alpha=0.4); fig.tight_layout(); save(fig, fname)

def chart_gap(results, fname):
    dc, ac = results['Dijkstra']['costs'], results['ACO']['costs']
    n = min(len(dc), len(ac))
    if n == 0: print("  Skipping gap chart."); return
    gaps = [((ac[i]-dc[i])/max(dc[i],1))*100 for i in range(n)]
    fig, ax = plt.subplots(figsize=(10, 5))
    cols = ['#34d399' if g <= 0 else '#f87171' for g in gaps]
    ax.bar(range(1, n+1), gaps, color=cols, alpha=0.8, width=0.7)
    ax.axhline(np.mean(gaps), color='#fbbf24', linewidth=1.8, linestyle='--', label=f'Mean gap: {np.mean(gaps):.1f}%')
    ax.axhline(0, color='#94a3b8', linewidth=1, linestyle='-', label='Optimal (Dijkstra)')
    ax.set_xlabel('OD Pair Index'); ax.set_ylabel('Cost Overhead vs Dijkstra (%)')
    ax.set_title('ACO Optimality Gap vs Dijkstra', fontsize=14, fontweight='bold', pad=14)
    ax.legend(framealpha=0.25, facecolor='#0f172a', edgecolor='#1e293b')
    ax.grid(True, alpha=0.3, axis='y'); ax.set_axisbelow(True); fig.tight_layout(); save(fig, fname)

def chart_success(results, n, fname):
    algos = list(COLORS.keys())
    rates = [results[a]['found']/n*100 for a in algos]
    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(algos, rates, color=[COLORS[a] for a in algos], width=0.5)
    for b, v in zip(bars, rates):
        ax.text(b.get_x()+b.get_width()/2, b.get_height()+0.5, f"{v:.0f}%", ha='center', va='bottom', fontsize=11, fontweight='bold')
    ax.set_ylim(0, 115); ax.set_title('Path Found Rate', fontsize=14, fontweight='bold', pad=14)
    ax.set_ylabel('% of OD Pairs Solved'); ax.yaxis.grid(True, alpha=0.4); ax.set_axisbelow(True)
    fig.tight_layout(); save(fig, fname)

def chart_summary(results, fname):
    algos = list(COLORS.keys()); rows = []
    for a in algos:
        r = results[a]
        rows.append([a,
            f"{np.mean(r['times']):.2f} ms" if r['times'] else '—',
            f"{np.std(r['times']):.2f} ms" if r['times'] else '—',
            f"{np.mean(r['costs']):.1f} s" if r['costs'] else '—',
            f"{np.mean(r['lengths']):.1f}" if r['lengths'] else '—',
            f"{r['found']}/{len(r['times']) or '?'}"])
    fig, ax = plt.subplots(figsize=(12, 3)); ax.axis('off')
    tbl = ax.table(cellText=rows, colLabels=['Algorithm','Mean Time','Std Time','Mean Path Cost','Mean Path Len','Paths Found'],
                   loc='center', cellLoc='center')
    tbl.auto_set_font_size(False); tbl.set_fontsize(11); tbl.scale(1, 2.2)
    for (row, col), cell in tbl.get_celld().items():
        cell.set_edgecolor('#1e293b')
        if row == 0: cell.set_facecolor('#1e3a5f'); cell.set_text_props(color='#f1f5f9', fontweight='bold')
        else:
            cell.set_facecolor('#0f172a' if row%2==0 else '#111827')
            if col == 0: cell.set_facecolor(COLORS[algos[row-1]]); cell.set_text_props(color='#020817', fontweight='bold')
            else: cell.set_text_props(color='#cbd5e1')
    ax.set_title('Algorithm Comparison — Summary (Real Congestion Data)', fontsize=13, fontweight='bold', pad=20)
    fig.tight_layout(); save(fig, fname)

# ═══════════════════ MAIN ═══════════════════

def main():
    lengths, centroids = load_network(NET_PATH)
    graph, routes = load_routes(ROUTES_PATH, lengths)
    if not routes: print("ERROR: No routes."); return
    cong_snapshots = load_congestion(TRAJ_PATH)

    random.shuffle(routes)
    od = []
    for r in routes:
        if len(r) >= 5:
            o, d = r[0], r[-1]
            if o != d and o in graph and d in lengths: od.append((o, d))
        if len(od) >= N_OD: break
    if not od: print("ERROR: No OD pairs."); return
    print(f"\n{len(od)} OD pairs selected. Graph: {len(graph)} nodes.")

    results = run_benchmark(graph, lengths, centroids, od, cong_snapshots)

    print("\n" + "="*62)
    print(f"{'Algorithm':<15} {'Mean Time':>12} {'Mean Cost':>12} {'Path Len':>10} {'Found':>8}")
    print("-"*62)
    for a in COLORS:
        r = results[a]
        t_str = f"{np.mean(r['times']):.2f} ms" if r['times'] else '—'
        c_str = f"{np.mean(r['costs']):.1f} s" if r['costs'] else '—'
        l_str = f"{np.mean(r['lengths']):.1f}" if r['lengths'] else '—'
        print(f"{a:<15} {t_str:>12} {c_str:>12} {l_str:>10} {r['found']}/{len(od):>7}")
    print("="*62)

    print("\nGenerating charts...")
    chart_bar(results, 'times', 'Average Execution Time per Algorithm', 'Time (ms)', '01_avg_execution_time.png')
    chart_box(results, 'times', 'Execution Time Distribution', 'Time (ms)', '02_time_distribution.png')
    chart_bar(results, 'costs', 'Average Path Cost (Congestion-Aware Travel Time)', 'Travel Time (s)', '03_avg_path_cost.png')
    chart_box(results, 'costs', 'Path Cost Distribution', 'Travel Time (s)', '04_cost_distribution.png')
    chart_scatter(results, '05_time_vs_cost_scatter.png')
    chart_gap(results, '06_aco_optimality_gap.png')
    chart_bar(results, 'lengths', 'Average Path Length (Edges)', 'Number of Edges', '07_path_length.png')
    chart_success(results, len(od), '08_success_rate.png')
    chart_summary(results, '09_summary_table.png')
    print(f"\nAll charts saved to: {FIG_DIR}")

if __name__ == '__main__':
    main()
