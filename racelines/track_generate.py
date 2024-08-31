import csv
import numpy as np

def generate_circular_path(radius, num_points):
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x_coords = radius * np.cos(angles) - radius
    y_coords = radius * np.sin(angles)
    
    # Rotate the points so the path starts at (0, 0)
    shift_index = np.argmin(np.sqrt(x_coords**2 + y_coords**2))
    x_coords = np.roll(x_coords, -shift_index)
    y_coords = np.roll(y_coords, -shift_index)
    
    rotation_matrix = np.array([[0, 1], [-1, 0]])
    x_coords, y_coords = np.dot(rotation_matrix, np.array([x_coords, y_coords]))
    
    return x_coords, y_coords

def write_to_csv(x_coords, y_coords, filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y"])
        for x, y in zip(x_coords, y_coords):
            writer.writerow([x, y])

output_directory = '/home/emin/eufs/src/pure_pursuit/racelines/velocity_tracks'
filename = f'{output_directory}/circular_path.csv'
radius = 40  # Radius of the circle
num_points = 150  # Number of points to generate

x_coords, y_coords = generate_circular_path(radius, num_points)
write_to_csv(x_coords, y_coords, filename)

print("CSV file has been created.")
