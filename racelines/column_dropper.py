import pandas as pd

# Load the CSV file
df = pd.read_csv('src/pure_pursuit/racelines/velocity_tracks/handling_track.csv')

# Keep only the first two columns
df = df.iloc[:, :2]

# Save the modified DataFrame back to a CSV file
df.to_csv('src/pure_pursuit/racelines/velocity_tracks/handling_track.csv', index=False)