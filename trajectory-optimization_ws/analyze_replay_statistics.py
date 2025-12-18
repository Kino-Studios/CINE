#!/usr/bin/env python3
"""
Analyze replay trajectory data to compare baseline vs constrained runs.
Shows statistical consistency and filter effectiveness.
"""

import pandas as pd
import numpy as np
from pathlib import Path
import glob


def load_replay_runs(pattern):
    """Load all CSV files matching pattern and interpolate to common time grid"""
    files = sorted(glob.glob(pattern))
    
    if not files:
        return None, []
    
    print(f"Found {len(files)} files matching {pattern}")
    
    # Load all files
    dfs = []
    for f in files:
        df = pd.read_csv(f)
        if len(df) > 0:
            dfs.append(df)
    
    if not dfs:
        return None, []
    
    # Find common time range
    min_time = max(df['time'].min() for df in dfs)
    max_time = min(df['time'].max() for df in dfs)
    
    # Create common time grid at 50 Hz
    common_time = np.arange(min_time, max_time, 0.02)
    
    print(f"  Common time range: {min_time:.3f} to {max_time:.3f}s ({len(common_time)} points)")
    
    # Interpolate all runs to common grid
    interpolated_data = []
    
    for df in dfs:
        run_data = {'time': common_time}
        for col in df.columns:
            if col != 'time':
                run_data[col] = np.interp(common_time, df['time'], df[col])
        interpolated_data.append(pd.DataFrame(run_data))
    
    # Combine into single DataFrame with mean and std
    combined = pd.DataFrame({'time': common_time})
    
    # For each variable, compute mean and std across all runs
    for col in dfs[0].columns:
        if col == 'time':
            continue
        
        # Stack all runs for this variable
        values = np.column_stack([run[col].values for run in interpolated_data])
        
        combined[f'{col}_mean'] = values.mean(axis=1)
        combined[f'{col}_std'] = values.std(axis=1)
        combined[f'{col}_min'] = values.min(axis=1)
        combined[f'{col}_max'] = values.max(axis=1)
    
    return combined, interpolated_data


def analyze_consistency(interpolated_runs, name):
    """Analyze how consistent the trajectories are across runs"""
    if not interpolated_runs or len(interpolated_runs) < 2:
        print(f"  Not enough runs for consistency analysis (need at least 2)")
        return
    
    # Calculate position variance at each time step
    positions = []
    speeds = []
    
    for run in interpolated_runs:
        x = run['planned_x'].values
        y = run['planned_y'].values
        z = run['planned_z'].values
        speed = run['planned_speed'].values
        
        positions.append(np.column_stack([x, y, z]))
        speeds.append(speed)
    
    positions = np.array(positions)  # (num_runs, num_timesteps, 3)
    speeds = np.array(speeds)  # (num_runs, num_timesteps)
    
    # Position std at each timestep, averaged over time
    position_std = positions.std(axis=0)  # (num_timesteps, 3)
    mean_x_std = position_std[:, 0].mean()
    mean_y_std = position_std[:, 1].mean()
    mean_z_std = position_std[:, 2].mean()
    mean_total_std = np.linalg.norm(position_std, axis=1).mean()
    
    # Speed std averaged over time
    speed_std = speeds.std(axis=0).mean()
    
    print(f"\n{name} Consistency (std across {len(interpolated_runs)} runs):")
    print(f"  Position std: X={mean_x_std:.4f}m, Y={mean_y_std:.4f}m, Z={mean_z_std:.4f}m")
    print(f"  Total position std: {mean_total_std:.4f}m")
    print(f"  Speed std: {speed_std:.4f} m/s")
    
    return {
        'x_std': mean_x_std,
        'y_std': mean_y_std,
        'z_std': mean_z_std,
        'total_std': mean_total_std,
        'speed_std': speed_std
    }


def main():
    data_dir = Path('./trajectory_data')
    
    print("=" * 60)
    print("Replay Trajectory Analysis")
    print("=" * 60)
    
    # Load baseline replay runs
    baseline_pattern = str(data_dir / 'baseline_replay_*.csv')
    baseline_stats, baseline_runs = load_replay_runs(baseline_pattern)
    
    # Load constrained replay runs
    constrained_pattern = str(data_dir / 'constrained_replay_*.csv')
    constrained_stats, constrained_runs = load_replay_runs(constrained_pattern)
    
    # Analyze consistency
    if baseline_runs:
        baseline_consistency = analyze_consistency(baseline_runs, "BASELINE")
    
    if constrained_runs:
        constrained_consistency = analyze_consistency(constrained_runs, "CONSTRAINED")
    
    # Save statistics if we have data
    if baseline_stats is not None:
        output_file = data_dir / 'baseline_replay_statistics.csv'
        baseline_stats.to_csv(output_file, index=False)
        print(f"\n✓ Baseline statistics saved to: {output_file}")
        
        # Print overall stats
        print(f"\nBaseline Overall Statistics:")
        print(f"  Mean speed: {baseline_stats['planned_speed_mean'].mean():.3f} m/s")
        print(f"  Max speed: {baseline_stats['planned_speed_max'].max():.3f} m/s")
        print(f"  Duration: {baseline_stats['time'].max() - baseline_stats['time'].min():.3f}s")
    
    if constrained_stats is not None:
        output_file = data_dir / 'constrained_replay_statistics.csv'
        constrained_stats.to_csv(output_file, index=False)
        print(f"\n✓ Constrained statistics saved to: {output_file}")
        
        # Print overall stats
        print(f"\nConstrained Overall Statistics:")
        print(f"  Mean speed: {constrained_stats['planned_speed_mean'].mean():.3f} m/s")
        print(f"  Max speed: {constrained_stats['planned_speed_max'].max():.3f} m/s")
        print(f"  Duration: {constrained_stats['time'].max() - constrained_stats['time'].min():.3f}s")
    
    # Compare if we have both
    if baseline_stats is not None and constrained_stats is not None:
        print("\n" + "=" * 60)
        print("Baseline vs Constrained Comparison")
        print("=" * 60)
        
        # Compare speeds
        baseline_mean_speed = baseline_stats['planned_speed_mean'].mean()
        constrained_mean_speed = constrained_stats['planned_speed_mean'].mean()
        speed_reduction = baseline_mean_speed - constrained_mean_speed
        speed_reduction_pct = (speed_reduction / baseline_mean_speed) * 100
        
        print(f"\nSpeed:")
        print(f"  Baseline: {baseline_mean_speed:.3f} m/s")
        print(f"  Constrained: {constrained_mean_speed:.3f} m/s")
        print(f"  Reduction: {speed_reduction:.3f} m/s ({speed_reduction_pct:.1f}%)")
        
        # Compare consistency (if available)
        if baseline_runs and constrained_runs and len(baseline_runs) >= 2 and len(constrained_runs) >= 2:
            print(f"\nConsistency Improvement:")
            print(f"  Position std: {baseline_consistency['total_std']:.4f}m → {constrained_consistency['total_std']:.4f}m")
            consistency_improvement = (baseline_consistency['total_std'] - constrained_consistency['total_std']) / baseline_consistency['total_std'] * 100
            print(f"  Improvement: {consistency_improvement:.1f}%")
            
            print(f"\n  Speed std: {baseline_consistency['speed_std']:.4f}m/s → {constrained_consistency['speed_std']:.4f}m/s")
            speed_consistency_improvement = (baseline_consistency['speed_std'] - constrained_consistency['speed_std']) / baseline_consistency['speed_std'] * 100
            print(f"  Improvement: {speed_consistency_improvement:.1f}%")
    
    print("\n" + "=" * 60)


if __name__ == '__main__':
    main()
