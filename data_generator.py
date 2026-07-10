import numpy as np
import pandas as pd
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import warnings

# Suppress matplotlib warnings for cleaner console output
warnings.filterwarnings("ignore") 

# ==========================================
# Core Classes
# ==========================================

class Target:
    """ Represents a physical maneuvering target (CTRV Model via Euler Integration) """
    def __init__(self, target_id: int, start_time: float, end_time: float, pos0: list, vel0: list, maneuvers: list, duration: float):
        self.id = target_id
        self.start_time = start_time
        self.end_time = end_time
        self.dt = 0.01 
        self.times = np.arange(0, duration + self.dt, self.dt)
        self.states = np.zeros((len(self.times), 6)) 
        
        pos = np.array(pos0, dtype=float)
        vel = np.array(vel0, dtype=float)
        
        maneuver_idx = 0
        num_maneuvers = len(maneuvers) if maneuvers else 0
        
        for i, t in enumerate(self.times):
            yaw = np.arctan2(vel[1], vel[0])
            speed = np.linalg.norm(vel[:2])
            z_vel = vel[2]
            
            turn_rate, lon_accel, z_accel = 0.0, 0.0, 0.0
            
            if maneuver_idx < num_maneuvers:
                man = maneuvers[maneuver_idx]
                if man.get('start_time', 0.0) <= t <= man.get('end_time', duration):
                    turn_rate = np.deg2rad(man.get('turn_rate_deg', 0.0))
                    lon_accel = man.get('lon_accel', 0.0)
                    z_accel = man.get('z_accel', 0.0)
                elif t > man.get('end_time', duration):
                    maneuver_idx += 1
            
            yaw += turn_rate * self.dt
            speed += lon_accel * self.dt
            z_vel += z_accel * self.dt
            
            vel[0] = speed * np.cos(yaw)
            vel[1] = speed * np.sin(yaw)
            vel[2] = z_vel
            pos = pos + vel * self.dt
            
            self.states[i, :3] = pos
            self.states[i, 3:] = vel

    def get_state_at(self, t: float):
        if self.start_time <= t <= self.end_time:
            state = np.zeros(6)
            for i in range(6):
                state[i] = np.interp(t, self.times, self.states[:, i])
            return state
        return None

class Sensor:
    """ Base Class for all physical sensors """
    def __init__(self, name: str, config: dict, rng: np.random.Generator):
        self.name = name
        self.rate_hz = config['rate_hz']
        self.drop_prob = config['drop_probability']
        self.clutter_lambda = config.get('clutter_lambda', 0.0)
        self.noise = config['noise_std']
        self.bias = config.get('bias', {})
        self.rng = rng

    def get_num_clutter(self) -> int:
        return self.rng.poisson(self.clutter_lambda)

    def generate_measurements(self, targets: list, duration: float) -> pd.DataFrame:
        raise NotImplementedError("Subclasses must implement this method")

class RadarSensor(Sensor):
    def generate_measurements(self, targets: list, duration: float) -> pd.DataFrame:
        timestamps = np.arange(0, duration, 1.0 / self.rate_hz)
        data = []
        
        b_r = self.bias.get('range_m', 0.0)
        b_az = np.deg2rad(self.bias.get('azimuth_deg', 0.0))
        b_el = np.deg2rad(self.bias.get('elevation_deg', 0.0))
        
        for t in timestamps:
            for target in targets:
                state = target.get_state_at(t)
                if state is None:
                    continue 
                    
                if self.rng.random() < self.drop_prob:
                    continue 
                    
                x, y, z, vx, vy, vz = state
                r = np.sqrt(x**2 + y**2 + z**2)
                az = np.arctan2(y, x)
                el = np.arcsin(z / r)
                vr = (x*vx + y*vy + z*vz) / r
                
                meas_r = r + self.rng.normal(0, self.noise['range_m']) + b_r
                meas_az = az + self.rng.normal(0, np.deg2rad(self.noise['azimuth_deg'])) + b_az
                meas_el = el + self.rng.normal(0, np.deg2rad(self.noise['elevation_deg'])) + b_el
                meas_vr = vr + self.rng.normal(0, self.noise['radial_vel_ms'])
                
                data.append({
                    'timestamp': round(t, 3),
                    'sensor': 'RADAR',
                    'target_id': target.id, 
                    'range': round(meas_r, 3),
                    'azimuth': round(np.rad2deg(meas_az), 3),
                    'elevation': round(np.rad2deg(meas_el), 3),
                    'radial_velocity': round(meas_vr, 3)
                })
                
            for _ in range(self.get_num_clutter()):
                c_r = self.rng.uniform(1000, 15000) 
                c_az = self.rng.uniform(0, np.pi)   
                c_el = self.rng.uniform(0, np.pi/4) 
                c_vr = self.rng.uniform(-200, 200)
                
                data.append({
                    'timestamp': round(t, 3),
                    'sensor': 'RADAR',
                    'target_id': 0, 
                    'range': round(c_r, 3),
                    'azimuth': round(np.rad2deg(c_az), 3),
                    'elevation': round(np.rad2deg(c_el), 3),
                    'radial_velocity': round(c_vr, 3)
                })
                
        return pd.DataFrame(data).sort_values(by='timestamp')

class EOSensor(Sensor):
    def generate_measurements(self, targets: list, duration: float) -> pd.DataFrame:
        timestamps = np.arange(0, duration, 1.0 / self.rate_hz)
        data = []
        
        b_az = np.deg2rad(self.bias.get('azimuth_deg', 0.0))
        b_el = np.deg2rad(self.bias.get('elevation_deg', 0.0))
        
        for t in timestamps:
            for target in targets:
                state = target.get_state_at(t)
                if state is None:
                    continue
                    
                if self.rng.random() < self.drop_prob:
                    continue
                    
                x, y, z = state[:3]
                r = np.sqrt(x**2 + y**2 + z**2)
                az = np.arctan2(y, x)
                el = np.arcsin(z / r)
                
                meas_az = az + self.rng.normal(0, np.deg2rad(self.noise['azimuth_deg'])) + b_az
                meas_el = el + self.rng.normal(0, np.deg2rad(self.noise['elevation_deg'])) + b_el
                
                data.append({
                    'timestamp': round(t, 3),
                    'sensor': 'EO',
                    'target_id': target.id,
                    'azimuth': round(np.rad2deg(meas_az), 3),
                    'elevation': round(np.rad2deg(meas_el), 3)
                })
                
            for _ in range(self.get_num_clutter()):
                c_az = self.rng.uniform(0, np.pi)
                c_el = self.rng.uniform(0, np.pi/4)
                
                data.append({
                    'timestamp': round(t, 3),
                    'sensor': 'EO',
                    'target_id': 0,
                    'azimuth': round(np.rad2deg(c_az), 3),
                    'elevation': round(np.rad2deg(c_el), 3)
                })
                
        return pd.DataFrame(data).sort_values(by='timestamp')

# ==========================================
# Visualization Modules
# ==========================================

def get_target_colors(truth_df):
    unique_targets = [tid for tid in truth_df['target_id'].unique() if tid != 0]
    cmap = plt.get_cmap('tab10')
    return {tid: cmap(i % 10) for i, tid in enumerate(unique_targets)}, unique_targets

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
        # Sensor Space (Range, Azimuth, Elevation)
        ax_rad.set_xlim([0, 20000]) # Max Range ~20km
        ax_rad.set_ylim([0, 180])   # Azimuth 0-180
        ax_rad.set_zlim([0, 45])    # Elevation 0-45
        ax_rad.set_xlabel('Range [m]')
        ax_rad.set_ylabel('Azimuth [deg]')
        ax_rad.set_zlabel('Elevation [deg]')
        
    return fig, ax_eo, ax_rad

# --- Plotting Builders ---

def plot_gt_eo(ax, truth_df, colors, unique_targets):
    for target_id in unique_targets:
        t_df = truth_df[truth_df['target_id'] == target_id]
        r = np.sqrt(t_df['x']**2 + t_df['y']**2 + t_df['z']**2)
        az_deg = np.degrees(np.arctan2(t_df['y'], t_df['x']))
        el_deg = np.degrees(np.arcsin(t_df['z'] / r))
        
        ax.plot(az_deg, el_deg, color=colors[target_id], linewidth=1.5, label=f'GT T{target_id}')
        ax.scatter(az_deg.iloc[0], el_deg.iloc[0], color='lime', marker='o', s=80, edgecolor='black', zorder=5)
        ax.scatter(az_deg.iloc[-1], el_deg.iloc[-1], color='red', marker='X', s=200, edgecolor='black', zorder=6)
        ax.text(az_deg.iloc[0], el_deg.iloc[0], f' T{target_id}', color='black', fontweight='bold')

def plot_gt_xyz(ax, truth_df, colors, unique_targets):
    for target_id in unique_targets:
        t_df = truth_df[truth_df['target_id'] == target_id]
        ax.plot(t_df['x'], t_df['y'], t_df['z'], color=colors[target_id], linewidth=1.5, label=f'GT T{target_id}')
        ax.scatter(t_df['x'].iloc[0], t_df['y'].iloc[0], t_df['z'].iloc[0], color='lime', marker='o', s=80, edgecolor='black', zorder=5)
        ax.scatter(t_df['x'].iloc[-1], t_df['y'].iloc[-1], t_df['z'].iloc[-1], color='red', marker='X', s=200, edgecolor='black', zorder=6)
        ax.text(t_df['x'].iloc[0], t_df['y'].iloc[0], t_df['z'].iloc[0], f' T{target_id}', color='black', fontweight='bold')

def plot_gt_rae(ax, truth_df, colors, unique_targets):
    for target_id in unique_targets:
        t_df = truth_df[truth_df['target_id'] == target_id]
        r = np.sqrt(t_df['x']**2 + t_df['y']**2 + t_df['z']**2)
        az_deg = np.degrees(np.arctan2(t_df['y'], t_df['x']))
        el_deg = np.degrees(np.arcsin(t_df['z'] / r))
        
        ax.plot(r, az_deg, el_deg, color=colors[target_id], linewidth=1.5, label=f'GT T{target_id}')
        ax.scatter(r.iloc[0], az_deg.iloc[0], el_deg.iloc[0], color='lime', marker='o', s=80, edgecolor='black', zorder=5)
        ax.scatter(r.iloc[-1], az_deg.iloc[-1], el_deg.iloc[-1], color='red', marker='X', s=200, edgecolor='black', zorder=6)
        ax.text(r.iloc[0], az_deg.iloc[0], el_deg.iloc[0], f' T{target_id}', color='black', fontweight='bold')

def plot_meas_eo(ax, eo_df):
    if eo_df.empty: return
    hits = eo_df[eo_df['target_id'] != 0]
    clutter = eo_df[eo_df['target_id'] == 0]
    ax.scatter(hits['azimuth'], hits['elevation'], color='blue', s=20, alpha=0.6, label='EO Hits')
    if not clutter.empty:
        ax.scatter(clutter['azimuth'], clutter['elevation'], color='orange', marker='x', s=20, alpha=0.9, label='EO Clutter')

def plot_meas_rae(ax, radar_df):
    if radar_df.empty: return
    hits = radar_df[radar_df['target_id'] != 0]
    clutter = radar_df[radar_df['target_id'] == 0]
    ax.scatter(hits['range'], hits['azimuth'], hits['elevation'], color='darkorange', s=30, alpha=0.8, edgecolor='black', label='Radar Hits')
    if not clutter.empty:
        ax.scatter(clutter['range'], clutter['azimuth'], clutter['elevation'], color='orange', marker='x', s=20, alpha=0.9, label='Radar Clutter')

def plot_meas_xyz(ax, radar_df):
    if radar_df.empty: return
    hits = radar_df[radar_df['target_id'] != 0]
    clutter = radar_df[radar_df['target_id'] == 0]
    
    # Convert Hits
    r, az, el = hits['range'].values, np.deg2rad(hits['azimuth'].values), np.deg2rad(hits['elevation'].values)
    ax.scatter(r*np.cos(el)*np.cos(az), r*np.cos(el)*np.sin(az), r*np.sin(el), color='darkorange', edgecolor='black', s=30, alpha=0.8, label='Radar Hits')
    
    # Convert Clutter
    if not clutter.empty:
        cr, caz, cel = clutter['range'].values, np.deg2rad(clutter['azimuth'].values), np.deg2rad(clutter['elevation'].values)
        ax.scatter(cr*np.cos(cel)*np.cos(caz), cr*np.cos(cel)*np.sin(caz), cr*np.sin(cel), color='orange', marker='x', s=20, alpha=0.9, label='Radar Clutter')

# --- Figure Generators ---

def generate_all_figures(truth_df, radar_df, eo_df):
    colors, unique_targets = get_target_colors(truth_df)
    
    # -------------------------------------------------------------------------
    # Fig 1: Ground Truth Only (Left: EO Az/El, Right: RADAR XYZ)
    # -------------------------------------------------------------------------
    fig1, ax1_eo, ax1_rad = setup_figure(radar_is_xyz=True)
    plot_gt_eo(ax1_eo, truth_df, colors, unique_targets)
    plot_gt_xyz(ax1_rad, truth_df, colors, unique_targets)
    
    ax1_eo.set_title('Fig 1: Ground Truth Trajectories (Left: EO View)')
    ax1_rad.set_title('Fig 1: Ground Truth Trajectories (Right: 3D Physical Space)')
    ax1_eo.legend(fontsize='small')
    ax1_rad.legend(fontsize='small')
    fig1.tight_layout()
    fig1.savefig('data/fig1_ground_truth.png')
    print("[+] Saved data/fig1_ground_truth.png")
    
    # -------------------------------------------------------------------------
    # Fig 2: Measurements Only (Left: EO Az/El, Right: RADAR R/Az/El)
    # -------------------------------------------------------------------------
    fig2, ax2_eo, ax2_rad = setup_figure(radar_is_xyz=False)
    plot_meas_eo(ax2_eo, eo_df)
    plot_meas_rae(ax2_rad, radar_df)
    
    ax2_eo.set_title('Fig 2: Raw Measurements (Left: EO Az/El)')
    ax2_rad.set_title('Fig 2: Raw Measurements (Right: RADAR Sensor Space)')
    ax2_eo.legend(fontsize='small')
    ax2_rad.legend(fontsize='small')
    fig2.tight_layout()
    fig2.savefig('data/fig2_measurements.png')
    print("[+] Saved data/fig2_measurements.png")
    
    # -------------------------------------------------------------------------
    # Fig 3: Combined in Sensor Space (Left: EO Az/El, Right: RADAR R/Az/El)
    # -------------------------------------------------------------------------
    fig3, ax3_eo, ax3_rad = setup_figure(radar_is_xyz=False)
    plot_gt_eo(ax3_eo, truth_df, colors, unique_targets)
    plot_meas_eo(ax3_eo, eo_df)
    plot_gt_rae(ax3_rad, truth_df, colors, unique_targets)
    plot_meas_rae(ax3_rad, radar_df)
    
    ax3_eo.set_title('Fig 3: Combined GT + Meas (Left: EO Sensor Space)')
    ax3_rad.set_title('Fig 3: Combined GT + Meas (Right: RADAR Sensor Space)')
    fig3.tight_layout()
    fig3.savefig('data/fig3_combined_sensor_space.png')
    print("[+] Saved data/fig3_combined_sensor_space.png")
    
    # -------------------------------------------------------------------------
    # Fig 4: Physical Projection (Left: EO Az/El, Right: RADAR XYZ)
    # -------------------------------------------------------------------------
    fig4, ax4_eo, ax4_rad = setup_figure(radar_is_xyz=True)
    plot_gt_eo(ax4_eo, truth_df, colors, unique_targets)
    plot_meas_eo(ax4_eo, eo_df)
    plot_gt_xyz(ax4_rad, truth_df, colors, unique_targets)
    plot_meas_xyz(ax4_rad, radar_df)
    
    ax4_eo.set_title('Fig 4: Physical Projection (Left: EO View)')
    ax4_rad.set_title('Fig 4: Physical Projection (Right: 3D Physical Space XYZ)')
    fig4.tight_layout()
    fig4.savefig('data/fig4_combined_physical_space.png')
    print("[+] Saved data/fig4_combined_physical_space.png")

# ==========================================
# Main
# ==========================================
def load_config(config_path: str = "config.yaml") -> dict:
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def generate_truth_data(targets: list, duration: float, rate_hz: float) -> pd.DataFrame:
    timestamps = np.arange(0, duration, 1.0 / rate_hz)
    data = []
    for t in timestamps:
        for target in targets:
            state = target.get_state_at(t)
            if state is not None:
                x, y, z, vx, vy, vz = state
                data.append({
                    'timestamp': round(t, 3),
                    'target_id': target.id,
                    'x': x, 'y': y, 'z': z,
                    'vx': vx, 'vy': vy, 'vz': vz
                })
    return pd.DataFrame(data)

def main():
    print("--- Starting Ultimate Production-Grade Data Generation ---")
    config = load_config()
    rng = np.random.default_rng(config['simulation']['random_seed'])
    duration = config['simulation']['duration_sec']
    
    targets = []
    for t_conf in config['targets']:
        maneuvers = t_conf.get('maneuvers', [])
        start_t = t_conf.get('start_time', 0.0)
        end_t = t_conf.get('end_time', duration)
        targets.append(Target(t_conf['id'], start_t, end_t, t_conf['initial_position'], t_conf['initial_velocity'], maneuvers, duration))
        
    truth_df = generate_truth_data(targets, duration, config['simulation']['truth_rate_hz'])
    truth_df.to_csv('data/ground_truth.csv', index=False)
    
    radar = RadarSensor("RADAR", config['sensors']['radar'], rng)
    radar_df = radar.generate_measurements(targets, duration)
    radar_export = radar_df.drop(columns=['target_id'])
    radar_export.to_csv('data/radar.csv', index=False)
    
    eo = EOSensor("EO", config['sensors']['eo'], rng)
    eo_df = eo.generate_measurements(targets, duration)
    eo_export = eo_df.drop(columns=['target_id'])
    eo_export.to_csv('data/eo.csv', index=False)

    # NEW: Export Noise Config for C++ Kalman Filter
    radar_noise = config['sensors']['radar']['noise_std']
    eo_noise = config['sensors']['eo']['noise_std']
    with open('data/sensor_noise.txt', 'w') as f:
        # Radar: range, az, el, vr
        f.write(f"{radar_noise['range_m']} {radar_noise['azimuth_deg']} {radar_noise['elevation_deg']} {radar_noise['radial_vel_ms']}\n")
        # EO: az, el
        f.write(f"{eo_noise['azimuth_deg']} {eo_noise['elevation_deg']}\n")
        
    print("[+] Exported Noise configuration to data/sensor_noise.txt")
    print("[+] Data generation complete. Generating Visualization Figures...")
    generate_all_figures(truth_df, radar_df, eo_df)
    print("--- Pipeline Execution Complete ---")

if __name__ == "__main__":
    main()