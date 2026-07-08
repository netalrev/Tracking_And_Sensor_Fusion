import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import warnings

warnings.filterwarnings("ignore")

def load_data():
    print("[+] Loading Data for Animation...")
    gt_df = pd.read_csv('data/ground_truth.csv')
    radar_df = pd.read_csv('data/radar.csv')
    fused_df = pd.read_csv('data/fused_tracks.csv')
    eo_df = pd.read_csv('data/eo.csv')
    
    # Convert Radar Spherical to Cartesian
    az_rad = np.radians(radar_df['azimuth'])
    el_rad = np.radians(radar_df['elevation'])
    r = radar_df['range']
    radar_df['x'] = r * np.cos(el_rad) * np.cos(az_rad)
    radar_df['y'] = r * np.cos(el_rad) * np.sin(az_rad)
    radar_df['z'] = r * np.sin(el_rad)
    
    return gt_df, radar_df, eo_df, fused_df

def animate_system():
    gt_df, radar_df, eo_df, fused_df = load_data()
    
    # Get all unique timestamps at 10Hz (0.1s steps)
    min_time = 0.0
    max_time = gt_df['timestamp'].max()
    timestamps = np.arange(min_time, max_time + 0.1, 0.1)
    
    fig = plt.figure(figsize=(20, 8))
    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122)
    
    cmap = plt.colormaps.get_cmap('tab10')

    def update(frame_idx):
        current_time = timestamps[frame_idx]
        
        ax1.clear()
        ax2.clear()
        
        # --- Setup Axes Limits & Titles ---
        ax1.set_title(f'3D World View (Time: {current_time:.1f}s)', fontsize=14)
        ax1.set_xlim([-5000, 15000])
        ax1.set_ylim([0, 15000])
        ax1.set_zlim([0, 10000])
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('Z [m]')
        
        ax2.set_title(f'2D Camera Screen (Time: {current_time:.1f}s)', fontsize=14)
        ax2.set_xlim([-5, 185])
        ax2.set_ylim([-5, 50])
        ax2.set_xlabel('Azimuth [deg]')
        ax2.set_ylabel('Elevation [deg]')
        ax2.grid(True, linestyle='--', alpha=0.5)

        # 1. Plot Ground Truth (History up to current time)
        gt_history = gt_df[gt_df['timestamp'] <= current_time]
        for tgt_id, group in gt_history.groupby('target_id'):
            ax1.plot(group['x'], group['y'], group['z'], 'k--', alpha=0.5, linewidth=1)

        # 2. Plot Sensor Hits (Blinking: Only show hits in the very recent past 0.2s)
        time_window = 0.2
        recent_radar = radar_df[(radar_df['timestamp'] <= current_time) & (radar_df['timestamp'] > current_time - time_window)]
        if not recent_radar.empty:
            ax1.scatter(recent_radar['x'], recent_radar['y'], recent_radar['z'], 
                        facecolors='none', edgecolors='orange', s=50, label='Radar Hits')

        recent_eo = eo_df[(eo_df['timestamp'] <= current_time) & (eo_df['timestamp'] > current_time - time_window)]
        if not recent_eo.empty:
            ax2.scatter(np.degrees(recent_eo['azimuth']), np.degrees(recent_eo['elevation']), 
                        color='red', marker='x', s=30, label='EO Hits')

        # 3. Plot Fused Tracks (History up to current time)
        fused_history = fused_df[fused_df['timestamp'] <= current_time]
        for trk_id, group in fused_history.groupby('target_id'):
            current_state = group.iloc[-1]['status'] # Check the latest status
            
            if current_state == "TENTATIVE":
                # Tentative tracks are gray and dashed
                color = 'gray'
                linestyle = ':'
                linewidth = 2
            else:
                # Confirmed tracks get standard colors
                color = cmap(trk_id % 10)
                linestyle = '-'
                linewidth = 3.5

            # Plot 3D
            ax1.plot(group['x'], group['y'], group['z'], color=color, linestyle=linestyle, linewidth=linewidth)
            
            # Plot 2D Camera View
            r = np.sqrt(group['x']**2 + group['y']**2 + group['z']**2)
            az_deg = np.degrees(np.arctan2(group['y'], group['x']))
            el_deg = np.degrees(np.arcsin(group['z'] / r))
            ax2.plot(az_deg, el_deg, color=color, linestyle=linestyle, linewidth=linewidth)
            
            # Add ID tag at the head of the track
            if current_state == "CONFIRMED":
                ax1.text(group['x'].iloc[-1], group['y'].iloc[-1], group['z'].iloc[-1], f' ID:{trk_id}', color=color, fontweight='bold')
                ax2.text(az_deg.iloc[-1], el_deg.iloc[-1], f' ID:{trk_id}', color=color, fontweight='bold')

    print(f"[+] Generating animation ({len(timestamps)} frames)... This might take a minute.")
    ani = FuncAnimation(fig, update, frames=len(timestamps), interval=100) # 100ms per frame = 10 FPS
    
    # Save as GIF
    output_path = 'data/tracking_animation.gif'
    ani.save(output_path, writer=PillowWriter(fps=10))
    print(f"[+] Animation saved successfully to '{output_path}'!")

if __name__ == "__main__":
    animate_system()