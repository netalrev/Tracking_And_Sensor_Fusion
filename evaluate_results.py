import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import warnings

# Suppress warnings for cleaner output
warnings.filterwarnings("ignore")

def load_data():
    print("[1] Loading Data...")
    gt_df = pd.read_csv('data/ground_truth.csv')
    radar_df = pd.read_csv('data/radar.csv')
    fused_df = pd.read_csv('data/fused_tracks.csv')
    eo_df = pd.read_csv('data/eo.csv')
    
    # Convert Radar Spherical to Cartesian for plotting
    az_rad = np.radians(radar_df['azimuth'])
    el_rad = np.radians(radar_df['elevation'])
    r = radar_df['range']
    
    radar_df['x'] = r * np.cos(el_rad) * np.cos(az_rad)
    radar_df['y'] = r * np.cos(el_rad) * np.sin(az_rad)
    radar_df['z'] = r * np.sin(el_rad)
    
    return gt_df, radar_df, eo_df, fused_df

def plot_trajectories(gt_df, radar_df, fused_df):
    print("[2] Generating 2D Trajectory Plot...")
    plt.figure(figsize=(12, 8))
    
    for tgt_id, group in gt_df.groupby('target_id'):
        plt.plot(group['x'], group['y'], label=f'GT Target {tgt_id}', linewidth=2, linestyle='--')
        
    plt.scatter(radar_df['x'], radar_df['y'], color='gray', s=10, alpha=0.3, label='Radar Clutter/Noise')
    
    for trk_id, group in fused_df.groupby('target_id'):
        plt.plot(group['x'], group['y'], label=f'Fused Track {trk_id}', linewidth=2, marker='o', markersize=3)
        plt.text(group['x'].iloc[0], group['y'].iloc[0], f' ID:{trk_id}', fontsize=12, fontweight='bold')

    plt.title('Sensor Fusion Results: 2D Target Trajectories', fontsize=16)
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend(loc='upper right', bbox_to_anchor=(1.25, 1))
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('data/evaluation_plot.png', dpi=300)

def evaluate_performance(gt_df, fused_df):
    print("\n[3] Evaluating Tracking Performance...")
    
    total_squared_error = 0
    total_points = 0
    
    for fused_id, f_group in fused_df.groupby('target_id'):
        start_time = f_group['timestamp'].min()
        f_start_pos = f_group.iloc[0][['x', 'y', 'z']].values
        
        gt_alive = gt_df[(gt_df['timestamp'] >= start_time - 0.1) & (gt_df['timestamp'] <= start_time + 0.1)]
        if gt_alive.empty: continue
            
        min_dist = float('inf')
        matched_gt_id = -1
        
        for gt_id, gt_group in gt_alive.groupby('target_id'):
            gt_pos = gt_group.iloc[0][['x', 'y', 'z']].values
            dist = np.linalg.norm(f_start_pos - gt_pos)
            if dist < min_dist:
                min_dist = dist
                matched_gt_id = gt_id
                
        gt_matched_group = gt_df[gt_df['target_id'] == matched_gt_id]
        merged = pd.merge_asof(f_group.sort_values('timestamp'), 
                               gt_matched_group.sort_values('timestamp'), 
                               on='timestamp', direction='nearest', 
                               suffixes=('_est', '_gt'))
        
        sq_errors = (merged['x_est'] - merged['x_gt'])**2 + \
                    (merged['y_est'] - merged['y_gt'])**2 + \
                    (merged['z_est'] - merged['z_gt'])**2
                    
        rmse = np.sqrt(sq_errors.mean())
        print(f"  -> RMSE for Fused Track {fused_id} (Matched to GT {matched_gt_id}): {rmse:.2f} meters")
        
        total_squared_error += sq_errors.sum()
        total_points += len(sq_errors)
        
    overall_rmse = np.sqrt(total_squared_error / total_points)
    print(f"\n[!] OVERALL SYSTEM RMSE: {overall_rmse:.2f} meters")

def plot_combined_visualization(gt_df, radar_df, eo_df, fused_df):
    print("\n[4] Generating 1-to-1 Apples-to-Apples Visualization...")
    
    # Create a 1x2 layout exactly like dataset_visualization_combined.png
    fig = plt.figure(figsize=(20, 8))
    
    # ==========================================
    # Subplot 1: 3D World View (Radar + Truth + Fused)
    # ==========================================
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_title('3D World View (Radar + Truth + Fused)', fontsize=14)
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    
    # CRITICAL: Lock axes exactly to the original generator's scale
    ax1.set_xlim([-15000, 15000])
    ax1.set_ylim([0, 15000])
    ax1.set_zlim([0, 10000])

    # 1. Plot GT
    for tgt_id, group in gt_df.groupby('target_id'):
        ax1.plot(group['x'], group['y'], group['z'], 'k--', alpha=0.8, linewidth=2.5, label=f'GT Target {tgt_id}', zorder=5)
    # 2. Plot Radar Clutter/Hits (Orange circles with no face color, exactly like original)
    ax1.scatter(radar_df['x'], radar_df['y'], radar_df['z'], 
                facecolors='none', edgecolors='orange', alpha=0.3, label='Radar Hits')

    # 3. Plot Fused Tracks
    cmap = plt.colormaps.get_cmap('tab10')
    for i, (trk_id, group) in enumerate(fused_df.groupby('target_id')):
        color = cmap(i % 10)
        ax1.plot(group['x'], group['y'], group['z'], color=color, linewidth=3, label=f'Fused Track {trk_id}')
        
    ax1.legend(loc='upper right')

    # ==========================================
    # Subplot 2: 2D Camera View (Azimuth vs Elevation)
    # ==========================================
    ax2 = fig.add_subplot(122)
    ax2.set_title('2D Camera View (EO Sensors + Fused Tracks)', fontsize=14)
    ax2.set_xlabel('Azimuth [deg]')
    ax2.set_ylabel('Elevation [deg]')
    ax2.grid(True, linestyle='--', alpha=0.7)
    
    # Lock axes exactly to the original camera's FOV
    ax2.set_xlim([-5, 185])
    ax2.set_ylim([-5, 50])


    # Plot ALL original EO Measurements (Make them red and clearly visible)
    ax2.scatter(np.degrees(eo_df['azimuth']), np.degrees(eo_df['elevation']), 
                color='red', marker='x', alpha=0.5, s=25, label='EO Raw Hits (Clutter+Targets)', zorder=1)

    # Map Fused Tracks back to Camera Space (Spherical angles)
    for i, (trk_id, group) in enumerate(fused_df.groupby('target_id')):
        color = cmap(i % 10)
        
        # Calculate R, Azimuth, and Elevation for the fused track
        r = np.sqrt(group['x']**2 + group['y']**2 + group['z']**2)
        az_deg = np.degrees(np.arctan2(group['y'], group['x']))
        el_deg = np.degrees(np.arcsin(group['z'] / r))
        
        ax2.plot(az_deg, el_deg, color=color, linewidth=3.5, label=f'Fused Track {trk_id}')
        # Add Track ID text next to the start of the line
        ax2.text(az_deg.iloc[0], el_deg.iloc[0], f' {trk_id}', color=color, fontsize=12, fontweight='bold')

    ax2.legend(loc='lower left', fontsize=9)

    plt.tight_layout()
    output_path = 'data/fused_visualization_apples_to_apples.png'
    plt.savefig(output_path, dpi=300)
    print(f"    -> Advanced Plot saved as '{output_path}'")

if __name__ == "__main__":
    print("=========================================")
    print("   Tracking System Evaluation Script")
    print("=========================================\n")
    
    gt, radar, eo, fused = load_data()
    plot_trajectories(gt, radar, fused)
    evaluate_performance(gt, fused)
    plot_combined_visualization(gt, radar, eo, fused)
    
    print("\n=========================================")
    print("             EVALUATION DONE")
    print("=========================================")