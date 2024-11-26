# 想看看读入的map的三个单独的txt文件是什么，在干嘛。
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_point_cloud(filename):
    """
    Read point cloud data file
    Return point cloud data in numpy array format
    """
    points = []
    try:
        with open(filename, 'r') as f:
            for line in f:
                # Filter empty lines
                if line.strip():
                    try:
                        # Split coordinates and convert to float
                        x, y, z = map(float, line.strip().split(','))
                        points.append([x, y, z])
                    except ValueError as e:
                        print(f"Warning: Skip invalid line: {line.strip()}")
                        continue
    except FileNotFoundError:
        print(f"Error: File not found {filename}")
        return None
    except Exception as e:
        print(f"Error: Error occurred while reading file: {str(e)}")
        return None

    if not points:
        print("Error: No valid point cloud data read")
        return None
    return np.array(points)

def visualize_point_cloud(points):
    """
    Visualize point cloud data
    Parameter points: point cloud data in numpy array format
    """
    if points is None or len(points) == 0:
        print("Error: No data to visualize")
        return

    # Set font properties
    plt.rcParams['font.family'] = 'DejaVu Sans'
    
    # Create new figure window
    plt.figure(figsize=(12, 8))
    ax = plt.axes(projection='3d')

    # Plot scatter points
    scatter = ax.scatter(points[:, 0],  # X coordinates
                        points[:, 1],  # Y coordinates
                        points[:, 2],  # Z coordinates
                        c=points[:, 2],  # Color by Z value
                        cmap='viridis',  # Use viridis colormap
                        marker='o',
                        s=50,  # Point size
                        alpha=0.6)  # Transparency

    # Add colorbar
    plt.colorbar(scatter, label='Z Coordinate', shrink=0.8)

    # Set axis labels
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')

    # Set title
    plt.title('3D Point Cloud Visualization')

    # Set initial view angle
    ax.view_init(elev=20, azim=45)

    # Add grid
    ax.grid(True)

    # Set equal aspect ratio
    ax.set_box_aspect([1,1,1])

    # Print statistics
    print("\nPoint Cloud Statistics:")
    print(f"Total points: {len(points)}")
    print("\nCoordinate Range:")
    print(f"X: {points[:, 0].min():.3f} to {points[:, 0].max():.3f}")
    print(f"Y: {points[:, 1].min():.3f} to {points[:, 1].max():.3f}")
    print(f"Z: {points[:, 2].min():.3f} to {points[:, 2].max():.3f}")

    # Show figure
    plt.show()

def main():
    """
    Main function
    """
    # Filename
    filename = '/home/johnnylin/AGLoc_ws/map/picking_list_star_center.txt'
    # Read point cloud data
    print(f"Reading file: {filename}")
    points = read_point_cloud(filename)
    
    if points is not None:
        print(f"Successfully read {len(points)} points")
        # Visualize point cloud
        visualize_point_cloud(points)
    else:
        print("Program terminated: Cannot read point cloud data")

if __name__ == "__main__":
    main()