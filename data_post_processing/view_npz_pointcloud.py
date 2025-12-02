import numpy as np
import open3d as o3d
import sys

def show_npz_xyz(path):
    """Load an NPZ file containing a (N,3) array named 'points' and display in Open3D."""
    data = np.load(path)
    
    # print(data['xyz'])
    if "xyz" not in data:
        raise KeyError("NPZ file must contain an array named 'points' with shape (N,3)")

    points = data["xyz"]
    points = np.vstack((points['x'], points['y'], points['z'])).T
    # points = np.asarray(points, dtype=np.float64)   # force float64

    # Ensure (N,3)
    points = points.reshape(-1, 3)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)

    o3d.visualization.draw_geometries([pc])


def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <path_to_npz>")
        sys.exit(1)

    npz_path = sys.argv[1]
    show_npz_xyz(npz_path)


if __name__ == "__main__":
    main()