#!/usr/bin/env python3

import pandas as pd
import numpy as np
from pathlib import Path

data_option = 0 # 0 for baseline, 1 for cinematic
current_data = "baseline" if data_option == 0 else "cinematic"

data_dir = Path(__file__).parent / 'trajectory_data'
csv_files = sorted(data_dir.glob(f"{current_data}_sim_*.csv"))

print(f"Found {len(csv_files)} {current_data} CSV files")

all_data = []
for csv_file in csv_files:
    df = pd.read_csv(csv_file)
    all_data.append(df)
    print(f"  {csv_file.name}: {len(df)} points, t={df['time'].min():.3f} to {df['time'].max():.3f}s")

min_time = max(df['time'].min() for df in all_data)
max_time = min(df['time'].max() for df in all_data)

common_time = np.arange(min_time, max_time, 0.02)

interpolated_data = []
columns = ['planned_x', 'planned_y', 'planned_z', 'planned_vx', 'planned_vy', 'planned_vz', 'planned_speed']

for df in all_data:
    interp_dict = {'time': common_time}
    for col in columns:
        interp_dict[col] = np.interp(common_time, df['time'], df[col])
    interpolated_data.append(pd.DataFrame(interp_dict))

stacked = np.stack([df[columns].values for df in interpolated_data], axis=0)

mean_values = np.mean(stacked, axis=0)
std_values = np.std(stacked, axis=0)

output_data = {'time': common_time}
for i, col in enumerate(columns):
    output_data[f'{col}_mean'] = mean_values[:, i]
    output_data[f'{col}_std'] = std_values[:, i]

output_df = pd.DataFrame(output_data)

mean_velocity = np.sqrt(
    output_df['planned_vx_mean']**2 + 
    output_df['planned_vy_mean']**2 + 
    output_df['planned_vz_mean']**2
).mean()

output_df['mean_velocity'] = mean_velocity

output_file = data_dir / f"{current_data}_statistics.csv"
output_df.to_csv(output_file, index=False)
