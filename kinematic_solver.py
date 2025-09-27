import numpy as np
from scipy.optimize import differential_evolution
from tqdm import tqdm
import pandas as pd
import warnings
warnings.filterwarnings('ignore')

# ----------------
# Globals
# ----------------
seeds = 4
l = 165.0  # mm

# ----------------
# Kinematic equation
# ----------------
def equation(theta_11, theta, l, l_12, l_11, h, w):
    a = np.arctan2(h, w)
    denom = np.hypot(w, h)
    lhs = (l_12**2 - l_11**2 - h**2 - w**2 - l**2) / denom
    rhs = (-2*l*np.cos(theta + a)
           + 2*l_11*np.cos(theta_11 + a)
           - (2*l*l_11/denom)*np.cos(theta - theta_11))
    return lhs - rhs

# ----------------
# Looser/faster sampling tuned to 0.1 mm precision
# ----------------
def find_working_geometries_0p1mm(num_samples,
                                  coverage_threshold,
                                  sampling_tol,   # DE convergence tolerance (coarse)
                                  accept_tol,      # accept if |fun| < 0.1 (mm-scale)
                                  seed0,
                                  theta11_deg_range=(-45, 90),
                                  theta_deg_bound=(-30, 30)):
    
    np.random.seed(seed0)
    results = []

    theta11_range = np.linspace(np.radians(theta11_deg_range[0]),
                                np.radians(theta11_deg_range[1]), 136)
    theta_bounds = [(np.radians(theta_deg_bound[0]), np.radians(theta_deg_bound[1]))]

    with tqdm(total=num_samples, desc="Sampling geometries (0.1mm)") as pbar:
        for _ in range(num_samples):
            # sample geometry; round to 0.1 mm
            w = float(np.random.randint(42, 82))
            h = float(np.random.randint(28, 68))
            l_11 = np.round(np.random.uniform(0.10*l, 0.30*l), 1)
            l_12 = np.round(np.random.uniform(0.25*l, 0.45*l), 1)

            successful_points = 0

            for theta_11_target in theta11_range:
                def obj(vars):
                    return equation(theta_11_target, vars[0], l, l_12, l_11, h, w)

                found = False
                for s in range(seeds):
                    try:
                        res = differential_evolution(
                            obj,
                            theta_bounds,
                            maxiter=100,   # fewer iterations
                            popsize=6,     # smaller population for speed
                            tol=sampling_tol,
                            seed=s
                        )
                        if res.success and abs(res.fun) < accept_tol:
                            found = True
                            break
                    except Exception:
                        continue

                if found:
                    successful_points += 1

            coverage = successful_points / len(theta11_range)
            if coverage >= coverage_threshold:
                results.append({
                    'w': int(w), 'h': int(h),
                    'l_11': float(l_11), 'l_12': float(l_12),
                    'coverage': coverage,
                    'successful_points': successful_points,
                    'total_points': len(theta11_range)
                })

            pbar.update(1)

    return results

# ----------------
# Quick helpers
# ----------------
def quick_report(results):
    if not results:
        print("No geometries found.")
        return
    print(f"Found {len(results)} geometries. coverage (min/max/mean): "
          f"{min(r['coverage'] for r in results):.3f}/"
          f"{max(r['coverage'] for r in results):.3f}/"
          f"{np.mean([r['coverage'] for r in results]):.3f}")
    for i, r in enumerate(sorted(results, key=lambda x: x['coverage'], reverse=True)[:10], start=1):
        print(f"{i}: w={r['w']}, h={r['h']}, l_11={r['l_11']:.1f}mm, l_12={r['l_12']:.1f}mm, cov={r['coverage']:.3f}")

def save_results(results, fname='working_geoms_0p1mm.csv'):
    if not results:
        print("No results to save.")
        return None
    df = pd.DataFrame(results)
    df.to_csv(fname, index=False)
    print(f"Saved {len(df)} rows to {fname}")
    return df

# ----------------
# Run
# ----------------
if __name__ == "__main__":
    res = find_working_geometries_0p1mm(num_samples=10000,
                                       coverage_threshold=0.90,
                                       sampling_tol=1.0,
                                       accept_tol=1.0,
                                       seed0=42)
    quick_report(res)
    df = save_results(res)
