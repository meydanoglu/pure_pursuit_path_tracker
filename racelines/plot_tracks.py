import pandas as pd
import matplotlib.pyplot as plt

# Step 1: Read the CSV file
file_path = '/home/emin/eufs/src/pure_pursuit/racelines/velocity_tracks/e7_floor5.csv'
df = pd.read_csv(file_path, header=None, names=['x', 'y', 'v'])

# Step 2: Extract coordinates
x = df['x']
y = df['y']

# Step 3: Plot the track
plt.figure()
plt.plot(x, y, label='Track')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('2D Track Plot')
plt.legend()

plt.show()