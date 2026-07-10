import os
import yaml
import subprocess
import shutil
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def load_exp_config():
    """
    Loads the experiment configuration from the YAML file.
    
    Returns:
        dict: The parsed 'experiment' dictionary from ../config/exp_config.yaml.
    """
    with open('../config/exp_config.yaml', 'r') as f:
        return yaml.safe_load(f)['experiment']

def backup_config():
    """
    Creates a temporary backup of the system configuration file prior to 
    sensitivity analysis overrides.
    """
    if os.path.exists('../config/config.yaml'):
        shutil.copy('../config/config.yaml', '../config/config_backup.yaml')

def restore_config():
    """
    Restores the original system configuration file after sensitivity tests 
    are completed.
    """
    if os.path.exists('../config/config_backup.yaml'):
        shutil.move('../config/config_backup.yaml', '../config/config.yaml')

def modify_config(multiplier, exp_target):
    multiplier = float(multiplier)
    with open('../config/config_backup.yaml', 'r') as f:
        config = yaml.safe_load(f)

    if exp_target in ['clutter', 'all']:
        config['sensors']['radar']['clutter_lambda'] *= multiplier
        config['sensors']['eo']['clutter_lambda'] *= multiplier

    if exp_target in ['noise', 'all']:
        for key in config['sensors']['radar']['noise_std']:
            config['sensors']['radar']['noise_std'][key] *= multiplier
        for key in config['sensors']['eo']['noise_std']:
            config['sensors']['eo']['noise_std'][key] *= multiplier

    if exp_target in ['drops', 'all']:
        config['sensors']['radar']['drop_probability'] = min(config['sensors']['radar']['drop_probability'] * multiplier, 0.50)
        config['sensors']['eo']['drop_probability'] = min(config['sensors']['eo']['drop_probability'] * multiplier, 0.50)

    with open('../config/config.yaml', 'w') as f:
        yaml.dump(config, f)

def run_full_pipeline():
    subprocess.run(["python", "data_generator.py"], check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["./tracking_system"], cwd="../build", check=True, stdout=subprocess.DEVNULL)

def extract_metrics():
    gt_df = pd.read_csv('../data/ground_truth.csv')
    fused_df = pd.read_csv('../results/fused_tracks.csv')
    
    if 'status' not in fused_df.columns:
        fused_df['status'] = 'CONFIRMED'

    confirmed_tracks = fused_df[fused_df['status'] == 'CONFIRMED']
    all_tracks = fused_df['target_id'].unique()
    graveyard_count = len([tid for tid in all_tracks if tid not in confirmed_tracks['target_id'].unique()])

    total_rmse_sq_sum = 0
    total_points = 0
    false_duration = 0
    
    init_delays = []
    overshoots = []

    if not confirmed_tracks.empty:
        for trk_id, f_group in confirmed_tracks.groupby('target_id'):
            f_start = f_group['timestamp'].min()
            f_end = f_group['timestamp'].max()
            f_start_pos = f_group.iloc[0][['x', 'y', 'z']].values

            matched_gt_id = -1
            min_dist = float('inf')
            for gt_id, gt_group in gt_df.groupby('target_id'):
                time_diffs = (gt_group['timestamp'] - f_start).abs()
                closest_idx = time_diffs.idxmin()
                if time_diffs[closest_idx] <= 2.0:
                    gt_pos = gt_group.loc[closest_idx, ['x', 'y', 'z']].values
                    dist = np.linalg.norm(f_start_pos - gt_pos)
                    if dist < min_dist:
                        min_dist = dist
                        matched_gt_id = gt_id

            if matched_gt_id == -1 or min_dist > 250.0:
                false_duration += (f_end - f_start)
                continue

            # Matched true track metrics
            gt_matched = gt_df[gt_df['target_id'] == matched_gt_id]
            gt_start = gt_matched['timestamp'].min()
            gt_end = gt_matched['timestamp'].max()
            
            init_delays.append(max(0.0, f_start - gt_start))
            overshoots.append(max(0.0, f_end - gt_end))

            merged = pd.merge_asof(f_group.sort_values('timestamp'),
                                   gt_matched.sort_values('timestamp'),
                                   on='timestamp', direction='nearest',
                                   suffixes=('_est', '_gt'))
            merged = merged[(merged['timestamp'] >= gt_start) &
                            (merged['timestamp'] <= gt_end)]
            
            if not merged.empty:
                err_sq = (merged['x_est'] - merged['x_gt'])**2 + \
                         (merged['y_est'] - merged['y_gt'])**2 + \
                         (merged['z_est'] - merged['z_gt'])**2
                total_rmse_sq_sum += err_sq.sum()
                total_points += len(merged)

    overall_rmse = np.sqrt(total_rmse_sq_sum / total_points) if total_points > 0 else 0.0
    avg_init_delay = np.mean(init_delays) if init_delays else 0.0
    avg_overshoot = np.mean(overshoots) if overshoots else 0.0

    return overall_rmse, false_duration, graveyard_count, avg_init_delay, avg_overshoot

def plot_separate_metrics(results, exp_target):
    difficulties = [r['difficulty'] for r in results]
    
    metrics_to_plot = [
        ('rmse', 'Overall RMSE [m]', 'tab:blue', 'RMSE'),
        ('overshoot', 'Average Overshoot Time [s]', 'tab:orange', 'Overshoot'),
        ('init_delay', 'Average Initialization Delay [s]', 'tab:green', 'Init_Delay'),
        ('graveyard', 'Tentative Graveyard [Tracks]', 'tab:gray', 'Graveyard'),
        ('false_duration', 'False Track Duration [s]', 'tab:red', 'False_Tracks')
    ]
    
    for key, ylabel, color, filename_suffix in metrics_to_plot:
        values = [r[key] for r in results]
        
        plt.figure(figsize=(10, 5))
        plt.plot(difficulties, values, marker='o', linewidth=2.5, color=color)
        plt.xlabel(f"Difficulty Multiplier (Target: {exp_target.upper()})")
        plt.ylabel(ylabel)
        plt.title(f"Sensitivity Analysis: {filename_suffix} vs. {exp_target.upper()}")
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.tight_layout()
        
        filename = f"../results/fig8_{exp_target}_metric_{filename_suffix.lower()}.png"
        plt.savefig(filename)
        plt.close()
        print(f"[+] Saved {filename}")

def main():
    exp_cfg = load_exp_config()
    exp_target = exp_cfg['target']

    print(f"\n{'='*65}")
    print(f" SENSITIVITY ANALYSIS: {exp_target.upper()} (5 ISOLATED METRICS)")
    print(f"{'='*65}\n")

    backup_config()

    start = exp_cfg.get('start_multiplier', 1.0)
    end = exp_cfg.get('max_multiplier', 15.0)
    step = exp_cfg.get('step_size', 1.0)
    difficulties = np.arange(start, end + step, step)

    results = []

    try:
        for diff in difficulties:
            diff_round = round(diff, 2)
            print(f"[*] Multiplier {diff_round}x...")

            modify_config(diff_round, exp_target)
            run_full_pipeline()

            rmse, false_dur, graveyard, init_del, oversh = extract_metrics()
            print(f"    RMSE: {rmse:.1f}m | False: {false_dur:.1f}s | Graveyard: {graveyard} | Init: {init_del:.1f}s | Over: {oversh:.1f}s")

            results.append({
                'difficulty': diff_round,
                'rmse': rmse,
                'false_duration': false_dur,
                'graveyard': graveyard,
                'init_delay': init_del,
                'overshoot': oversh
            })

    finally:
        restore_config()

    print("\n[+] Generating 5 separate metric plots...")
    plot_separate_metrics(results, exp_target)
    print("\n[+] Analysis Complete!")

if __name__ == "__main__":
    main()