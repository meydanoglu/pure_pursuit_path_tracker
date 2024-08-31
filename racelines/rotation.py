import csv
import numpy as np

def rotate_points(x_coords, y_coords):
    rotation_matrix = np.array([[0, 1], [-1, 0]]) # 90 degree rotation matrix
    rotated_coords = np.dot(rotation_matrix, np.array([x_coords, y_coords]))
    return rotated_coords[0], rotated_coords[1]

def rotate_csv(input_file, output_file):
    with open(input_file, mode='r') as infile:
        reader = csv.reader(infile)
        header = next(reader)
        rows = [row for row in reader]

    x_coords = [float(row[0]) for row in rows]
    y_coords = [float(row[1]) for row in rows]

    x_rotated, y_rotated = rotate_points(x_coords, y_coords)

    with open(output_file, mode='w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerow(header)
        for x, y in zip(x_rotated, y_rotated):
            writer.writerow([x, y])

input_file = 'src/pure_pursuit/racelines/velocity_tracks/handling_track.csv'
output_file = 'src/pure_pursuit/racelines/velocity_tracks/handling_track.csv'  # Overwrite the same file
rotate_csv(input_file, output_file)