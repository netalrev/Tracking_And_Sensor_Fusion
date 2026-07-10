import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import warnings

warnings.filterwarnings("ignore")

# ==========================================
# 1. Data Loading & Preprocessing
# ==========================================

def load_data():
    print("[+] Loading Tracking Data for Evaluation...")
    gt_df = pd.read_csv('data/ground_truth.csv')
    radar_df = pd.read_csv('data/radar.csv')
    eo_df = pd.read_csv('data/eo.csv')
    fused_df = pd.read_csv('data/fused_tracks.csv')
    
    # Ensure 'status' column exists in fused_df (Backward compatibility)
    if 'status' not in fused_df.columns:
        fused_df['status'] = 'CONFIRMED'
        
    return gt_df, radar_df, eo_df, fused_df

def get_target_colors(truth_df):
    unique_targets = [tid for tid in truth_df['target_id'].unique() if tid != 0]
    cmap = plt.get_cmap('tab10')
    return {tid: cmap(i % 10) for i, tid in enumerate(unique_targets)}, unique_targets

# ==========================================
# 2. Analytical Metrics & Evaluation
# ==========================================

def evaluate_metrics(gt_df, fused_df):
    print("\n" + "="*65)
    print("      ADVANCED TRACKING SYSTEM PERFORMANCE REPORT")
    print("="*65)
    
    # 1. Tentative Graveyard (Tracks that never reached CONFIRMED)
    all_track_ids = fused_df['target_id'].unique()
    confirmed_ids = fused_df[fused_df['status'] == 'CONFIRMED']['target_id'].unique()
    tentative_only_ids = [tid for tid in all_track_ids if tid not in confirmed_ids]
    print(f"[*] Tentative Graveyard (Tracks killed before confirmation): {len(tentative_only_ids)}")
    
    # Process Confirmed Tracks
    confirmed_tracks = fused_df[fused_df['status'] == 'CONFIRMED']
    if confirmed_tracks.empty:
        print("[!] No CONFIRMED tracks found to evaluate.")
        return []
        
    total_squared_error = 0
    total_points = 0
    track_metrics = [] # Critical for Fig 7 Plotting
    false_track_total_duration = 0
    
    for trk_id, f_group in confirmed_tracks.groupby('target_id'):
        f_start_time = f_group['timestamp'].min()
        f_end_time = f_group['timestamp'].max()
        f_start_pos = f_group.iloc[0][['x', 'y', 'z']].values
        
        # --- ROBUST GT MATCHING (Trajectory-Based) ---
        matched_gt_id = -1
        min_early_dist = float('inf')
        
        for gt_id, gt_group in gt_df.groupby('target_id'):
            gt_start_time = gt_group['timestamp'].min()
            gt_end_time = gt_group['timestamp'].max()
            
            # The GT must have existed around the time the track started
            if gt_end_time < f_start_time - 2.0 or gt_start_time > f_start_time + 5.0:
                continue
                
            merged = pd.merge_asof(f_group.sort_values('timestamp'), 
                                   gt_group.sort_values('timestamp'), 
                                   on='timestamp', direction='nearest', 
                                   tolerance=2.0, 
                                   suffixes=('_est', '_gt'))
            
            merged = merged.dropna(subset=['x_gt'])
            
            if len(merged) > 0:
                # Calculate average distance over the first 5 measurements to cancel out initial noise
                early_overlap = merged.head(5)
                early_dist = np.sqrt((early_overlap['x_est'] - early_overlap['x_gt'])**2 + 
                                     (early_overlap['y_est'] - early_overlap['y_gt'])**2 + 
                                     (early_overlap['z_est'] - early_overlap['z_gt'])**2).mean()
                
                if early_dist < min_early_dist:
                    min_early_dist = early_dist
                    matched_gt_id = gt_id
        
        # False Track Logic
        if matched_gt_id == -1 or min_early_dist > 250.0:
            duration = f_end_time - f_start_time
            false_track_total_duration += duration
            print(f"    [!] FALSE TRACK {trk_id} identified (Duration: {duration:.1f}s, Min Dist: {min_early_dist:.1f}m)")
            continue
            
        # Metric Logic for True Tracks
        gt_matched_group = gt_df[gt_df['target_id'] == matched_gt_id]
        gt_start_time = gt_matched_group['timestamp'].min()
        gt_end_time = gt_matched_group['timestamp'].max()
        
        # Lifecycle Metrics
        init_delay = max(0, f_start_time - gt_start_time)
        overshoot = max(0, f_end_time - gt_end_time)
        
        # Merge exactly based on nearest timestamp to compute error
        merged = pd.merge_asof(f_group.sort_values('timestamp'), 
                               gt_matched_group.sort_values('timestamp'), 
                               on='timestamp', direction='nearest', 
                               suffixes=('_est', '_gt'))
        
        # CUT-OFF FIX: Evaluate RMSE *only* when the GT is physically alive!
        merged = merged[(merged['timestamp'] >= gt_start_time) & (merged['timestamp'] <= gt_end_time)]
        
        if not merged.empty:
            # Calculate XYZ Position Error
            merged['pos_error'] = np.sqrt((merged['x_est'] - merged['x_gt'])**2 + 
                                          (merged['y_est'] - merged['y_gt'])**2 + 
                                          (merged['z_est'] - merged['z_gt'])**2)
            rmse = np.sqrt((merged['pos_error']**2).mean())
            total_squared_error += (merged['pos_error']**2).sum()
            total_points += len(merged)
        else:
            rmse = 0.0
            
        print(f"[*] Track {trk_id} <-> GT {matched_gt_id} | Init Delay: {init_delay:.1f}s | Overshoot: {overshoot:.1f}s | RMSE: {rmse:.1f}m")
        
        # Save metrics for plotting Fig 7
        track_metrics.append({
            'fused_id': trk_id,
            'gt_id': matched_gt_id,
            'merged_data': merged
        })
        
    print("-" * 65)
    overall_rmse = np.sqrt(total_squared_error / total_points) if total_points > 0 else 0.0
    print(f"[=>] OVERALL SYSTEM RMSE: {overall_rmse:.2f} meters")
    print(f"[=>] Total False Track Duration: {false_track_total_duration:.1f} seconds")
    print("=" * 65 + "\n")
    
    return track_metrics

# ==========================================
# 3. Visualization Tools (Apples-to-Apples)
# ==========================================

def setup_figure(radar_is_xyz=True):
    fig = plt.figure(figsize=(20, 8))
    # Left: EO (2D)
    ax_eo = fig.add_subplot(121)
    ax_eo.set_xlim([-5, 185])
    ax_eo.set_ylim([-5, 50])
    ax_eo.set_xlabel('Azimuth [deg]')
    ax_eo.set_ylabel('Elevation [deg]')
    ax_eo.grid(True, linestyle='--', alpha=0.6)
    
    # Right: RADAR (3D)
    ax_rad = fig.add_subplot(122, projection='3d')
    if radar_is_xyz:
        ax_rad.set_xlim([-5000, 15000])
        ax_rad.set_ylim([0, 15000])
        ax_rad.set_zlim([0, 10000])
        ax_rad.set_xlabel('X [m]')
        ax_rad.set_ylabel('Y [m]')
        ax_rad.set_zlabel('Z [m]')
    else:
        ax_rad.set_xlim([0, 20000]) 
        ax_rad.set_ylim([0, 180])   
        ax_rad.set_zlim([0, 45])    
        ax_rad.set_xlabel('Range [m]')
        ax_rad.set_ylabel('Azimuth [deg]')
        ax_rad.set_zlabel('Elevation [deg]')
        
    return fig, ax_eo, ax_rad

def plot_gt_reference(ax_eo, ax_rad, truth_df, colors, unique_targets, radar_is_xyz):
    """ Plots the clean GT as dashed lines in the background for reference """
    for target_id in unique_targets:
        t_df = truth_df[truth_df['target_id'] == target_id]
        color = colors[target_id]
        
        # Plot EO View (Az/El)
        r = np.sqrt(t_df['x']**2 + t_df['y']**2 + t_df['z']**2)
        az_deg = np.degrees(np.arctan2(t_df['y'], t_df['x']))
        el_deg = np.degrees(np.arcsin(t_df['z'] / r))
        ax_eo.plot(az_deg, el_deg, color=color, linewidth=1.5, linestyle='--', alpha=0.7, label=f'GT T{target_id}')
        
        # Plot Radar/Physical View
        if radar_is_xyz:
            ax_rad.plot(t_df['x'], t_df['y'], t_df['z'], color=color, linewidth=1.5, linestyle='--', alpha=0.7)
            ax_rad.scatter(0, 0, 0, color='black', marker='^', s=200) # Sensor Origin
        else:
            ax_rad.plot(r, az_deg, el_deg, color=color, linewidth=1.5, linestyle='--', alpha=0.7)

def plot_fused_tracks(ax_eo, ax_rad, fused_df, radar_is_xyz):
    """ Plots the estimated EKF Tracks """
    cmap = plt.get_cmap('Dark2')
    
    confirmed_tracks = fused_df[fused_df['status'] == 'CONFIRMED']
    
    for i, (trk_id, group) in enumerate(confirmed_tracks.groupby('target_id')):
        color = cmap(i % 8)
        
        # 1. Plot on EO View (Convert XYZ to Az/El)
        r = np.sqrt(group['x']**2 + group['y']**2 + group['z']**2)
        az_deg = np.degrees(np.arctan2(group['y'], group['x']))
        el_deg = np.degrees(np.arcsin(group['z'] / r))
        
        ax_eo.plot(az_deg, el_deg, color=color, linewidth=1.5, alpha=0.8, label=f'Track {trk_id}')
        ax_eo.scatter(az_deg.iloc[0], el_deg.iloc[0], color='lime', marker='o', s=80, edgecolor='black', zorder=5)
        ax_eo.scatter(az_deg.iloc[-1], el_deg.iloc[-1], color='red', marker='X', s=200, edgecolor='black', zorder=6)
        ax_eo.text(az_deg.iloc[0], el_deg.iloc[0], f' TRK {trk_id}', color='black', fontweight='bold')
        
        # 2. Plot on Radar/Physical View
        if radar_is_xyz:
            ax_rad.plot(group['x'], group['y'], group['z'], color=color, linewidth=1.5, alpha=0.8, label=f'Track {trk_id}')
            ax_rad.scatter(group['x'].iloc[0], group['y'].iloc[0], group['z'].iloc[0], color='lime', marker='o', s=80, edgecolor='black', zorder=5)
            ax_rad.scatter(group['x'].iloc[-1], group['y'].iloc[-1], group['z'].iloc[-1], color='red', marker='X', s=200, edgecolor='black', zorder=6)
            ax_rad.text(group['x'].iloc[0], group['y'].iloc[0], group['z'].iloc[0], f' TRK {trk_id}', color='black', fontweight='bold')
        else:
            ax_rad.plot(r, az_deg, el_deg, color=color, linewidth=1.5, alpha=0.8, label=f'Track {trk_id}')
            ax_rad.scatter(r.iloc[0], az_deg.iloc[0], el_deg.iloc[0], color='lime', marker='o', s=80, edgecolor='black', zorder=5)
            ax_rad.scatter(r.iloc[-1], az_deg.iloc[-1], el_deg.iloc[-1], color='red', marker='X', s=200, edgecolor='black', zorder=6)
            ax_rad.text(r.iloc[0], az_deg.iloc[0], el_deg.iloc[0], f' TRK {trk_id}', color='black', fontweight='bold')

def generate_evaluation_plots(truth_df, fused_df):
    print("[+] Generating Evaluation Figures...")
    colors, unique_targets = get_target_colors(truth_df)
    
    # -------------------------------------------------------------------------
    # Fig 5: Evaluated Tracks in Sensor Space (GT vs Track)
    # -------------------------------------------------------------------------
    fig5, ax5_eo, ax5_rad = setup_figure(radar_is_xyz=False)
    plot_gt_reference(ax5_eo, ax5_rad, truth_df, colors, unique_targets, radar_is_xyz=False)
    plot_fused_tracks(ax5_eo, ax5_rad, fused_df, radar_is_xyz=False)
    
    ax5_eo.set_title('Fig 5: EKF Tracks vs GT (Left: EO Sensor Space)')
    ax5_rad.set_title('Fig 5: EKF Tracks vs GT (Right: RADAR Sensor Space)')
    ax5_eo.legend(fontsize='small')
    fig5.tight_layout()
    fig5.savefig('data/fig5_eval_sensor_space.png')
    
    # -------------------------------------------------------------------------
    # Fig 6: Evaluated Tracks in Physical Space (XYZ)
    # -------------------------------------------------------------------------
    fig6, ax6_eo, ax6_rad = setup_figure(radar_is_xyz=True)
    plot_gt_reference(ax6_eo, ax6_rad, truth_df, colors, unique_targets, radar_is_xyz=True)
    plot_fused_tracks(ax6_eo, ax6_rad, fused_df, radar_is_xyz=True)
    
    ax6_eo.set_title('Fig 6: EKF Tracks vs GT (Left: EO View)')
    ax6_rad.set_title('Fig 6: EKF Tracks vs GT (Right: 3D Physical Space XYZ)')
    ax6_rad.legend(fontsize='small')
    fig6.tight_layout()
    fig6.savefig('data/fig6_eval_physical_space.png')

def plot_error_over_time(track_metrics):
    """ Fig 7: Plots the RMSE over time to show filter convergence """
    if not track_metrics: return
    
    plt.figure(figsize=(14, 6))
    
    for metrics in track_metrics:
        merged = metrics['merged_data']
        plt.plot(merged['timestamp'], merged['pos_error'], linewidth=2.5, label=f"Track {metrics['fused_id']} (GT {metrics['gt_id']})")
        
    plt.title('Fig 7: Position Error (RMSE) over Time', fontsize=16)
    plt.xlabel('Simulation Time [s]')
    plt.ylabel('Position Error [m]')
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    plt.savefig('data/fig7_rmse_over_time.png')
    print("[+] Saved data/fig7_rmse_over_time.png")

# ==========================================
# Main Execution
# ==========================================
def main():
    gt, radar, eo, fused = load_data()
    
    # 1. Analytics
    track_metrics = evaluate_metrics(gt, fused)
    
    # 2. Visualizations
    generate_evaluation_plots(gt, fused)
    if track_metrics:
        plot_error_over_time(track_metrics)
        
    print("\n[+] Single Run Evaluation Complete!")

if __name__ == "__main__":
    main()