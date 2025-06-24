import matplotlib.pyplot as plt
import os

def plot_distances(distances, out_dir):
    """
    Plot distance (loss) curve and save it.
    """
    plt.figure()
    plt.plot(distances, marker='o')
    plt.title('Distance to motif over steps')
    plt.xlabel('Step')
    plt.ylabel('Distance')
    plt.grid(True)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'distance_curve.png')
    plt.savefig(path)
    print(f"Distance curve saved to {path}")
