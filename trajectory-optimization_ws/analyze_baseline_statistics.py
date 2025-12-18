#!/usr/bin/env python3

import pandas as pd
import numpy as np
from pathlib import Path

# Find all baseline CSV files
data_dir = Path(__file__).parent / 'trajectory_data'
csv_files = sorted(data_dir.glob('baseline_sim_*.csv'))

print(f"Found {len(csv_files)} baseline CSV files")

# Read all CSVs
all_data = []
for csv_file in csv_files:
    df = pd.read_csv(csv_file)
    all_data.append(df)
    print(f"  {csv_file.name}: {len(df)} points, t={df['time'].min():.3f} to {df['time'].max():.3f}s")

# Find common time range (intersection of all runs)
min_time = max(df['time'].min() for df in all_data)
max_time = min(df['time'].max() for df in all_data)
print(f"\nCommon time range: {min_time:.3f} to {max_time:.3f}s")

# Create common time grid (50 Hz sampling)
common_time = np.arange(min_time, max_time, 0.02)
print(f"Common grid: {len(common_time)} points at 50 Hz")

# Interpolate all data to common time grid
interpolated_data = []
columns = ['planned_x', 'planned_y', 'planned_z', 'planned_vx', 'planned_vy', 'planned_vz', 'planned_speed']

for df in all_data:
    interp_dict = {'time': common_time}
    for col in columns:
        # Linear interpolation using numpy
        interp_dict[col] = np.interp(common_time, df['time'], df[col])
    interpolated_data.append(pd.DataFrame(interp_dict))

# Stack all runs and calculate statistics
stacked = np.stack([df[columns].values for df in interpolated_data], axis=0)  # shape: (n_runs, n_points, n_columns)

mean_values = np.mean(stacked, axis=0)  # shape: (n_points, n_columns)
std_values = np.std(stacked, axis=0)    # shape: (n_points, n_columns)

# Create output DataFrame
output_data = {'time': common_time}
for i, col in enumerate(columns):
    output_data[f'{col}_mean'] = mean_values[:, i]
    output_data[f'{col}_std'] = std_values[:, i]

output_df = pd.DataFrame(output_data)

# Calculate mean velocity magnitude across all trajectories
mean_velocity = np.sqrt(
    output_df['planned_vx_mean']**2 + 
    output_df['planned_vy_mean']**2 + 
    output_df['planned_vz_mean']**2
).mean()

output_df['mean_velocity'] = mean_velocity

# Save to CSV
output_file = data_dir / 'baseline_statistics.csv'
output_df.to_csv(output_file, index=False)

print(f"\nStatistics saved to: {output_file}")
print(f"  {len(output_df)} time points")
print(f"  Columns: time, {', '.join([f'{col}_mean, {col}_std' for col in columns])}")

# Print summary statistics
print("\n=== Summary Statistics ===")
print(f"Mean velocity (magnitude): {mean_velocity:.3f} m/s")
print(f"Mean speed: {output_df['planned_speed_mean'].mean():.3f} ± {output_df['planned_speed_std'].mean():.3f} m/s")
print(f"Max speed: {output_df['planned_speed_mean'].max():.3f} m/s")
print(f"Position std (x, y, z):")
print(f"  X: mean={output_df['planned_x_std'].mean():.3f}, max={output_df['planned_x_std'].max():.3f}")
print(f"  Y: mean={output_df['planned_y_std'].mean():.3f}, max={output_df['planned_y_std'].max():.3f}")
print(f"  Z: mean={output_df['planned_z_std'].mean():.3f}, max={output_df['planned_z_std'].max():.3f}")
