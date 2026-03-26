import numpy as np
import matplotlib.pyplot as plt
import glob, os, math, csv

files = sorted(glob.glob('hardware_results_test1/*.npz'), key=os.path.getmtime)
files = files[-2:]  
print(f"Using {len(files)} files:\n")

all_results = []

for f in files:
    d = np.load(f, allow_pickle=True)
    hist = d['car1_history']

    if hist.shape[0] == 0:
        print(f"  Skipping {os.path.basename(f)} — empty\n")
        continue

    x = hist[:, 0]
    y = hist[:, 1]
    theta_start = hist[0, 2]

    # Reconstruct circle center (same formula as shared2.py)
    radius = 1.5
    center_x = x[0] + radius * math.sin(theta_start)
    center_y = y[0] - radius * math.cos(theta_start)

    # Tracking error
    dist_from_center = np.sqrt((x - center_x)**2 + (y - center_y)**2)
    tracking_error = np.abs(dist_from_center - radius)

    result = {
        'file': os.path.basename(f),
        'n_points': len(x),
        'mean_error': float(np.mean(tracking_error)),
        'max_error': float(np.max(tracking_error)),
        'rmse': float(np.sqrt(np.mean(tracking_error**2))),
        'std_error': float(np.std(tracking_error)),
    }
    all_results.append(result)

    print(f"File: {result['file']}")
    print(f"  Points     : {result['n_points']}")
    print(f"  Mean error : {result['mean_error']:.4f} m")
    print(f"  Max error  : {result['max_error']:.4f} m")
    print(f"  RMSE       : {result['rmse']:.4f} m")
    print(f"  Std error  : {result['std_error']:.4f} m\n")

# Summary
if all_results:
    print("="*40)
    print("SUMMARY ACROSS LATEST 2 RUNS")
    print(f"  Mean error : {np.mean([r['mean_error'] for r in all_results]):.4f} m")
    print(f"  Mean RMSE  : {np.mean([r['rmse'] for r in all_results]):.4f} m")
    print(f"  Mean max   : {np.mean([r['max_error'] for r in all_results]):.4f} m")
    print(f"  Mean std   : {np.mean([r['std_error'] for r in all_results]):.4f} m")

    # Save CSV
    with open('tracking_errors_latest2.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=all_results[0].keys())
        writer.writeheader()
        writer.writerows(all_results)
    print("\nSaved to tracking_errors_latest2.csv")

   #plot
    fig, axes = plt.subplots(1, len(all_results), figsize=(4*len(all_results), 4))
    if len(all_results) == 1:
        axes = [axes]

    for ax, result, f in zip(axes, all_results, files):
        d = np.load(f, allow_pickle=True)
        hist = d['car1_history']
        x = hist[:, 0]
        y = hist[:, 1]
        theta_start = hist[0, 2]

        radius = 1.0
        center_x = x[0] + radius * math.sin(theta_start)
        center_y = y[0] - radius * math.cos(theta_start)

        theta = np.linspace(0, 2*math.pi, 200)
        ref_x = center_x + radius * np.cos(theta)
        ref_y = center_y + radius * np.sin(theta)

        ax.plot(ref_y, ref_x, 'r--', label='Reference')
        ax.plot(y, x, 'b-', label='Actual')
        ax.plot(y[0], x[0], 'go', markersize=8, label='Start')
        ax.set_title(f"{result['file'][:15]}\nRMSE:{result['rmse']:.3f}m")
        ax.axis('equal')
        ax.grid(True)
        ax.legend(fontsize=7)

    plt.tight_layout()
    plt.savefig('tracking_latest2.png')
    plt.show()
    print("Saved to tracking_latest2.png")
