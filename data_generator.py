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
        self.dt = 0.01 # 100Hz integration for maximum kinematic accuracy
        self.times = np.arange(0, duration + self.dt, self.dt)
        self.states = np.zeros((len(self.times), 6)) # State vector: [x, y, z, vx, vy, vz]
        
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
                # Maneuvers are based on absolute simulation time
                if man.get('start_time', 0.0) <= t <= man.get('end_time', duration):
                    turn_rate = np.deg2rad(man.get('turn_rate_deg', 0.0))
                    lon_accel = man.get('lon_accel', 0.0)
                    z_accel = man.get('z_accel', 0.0)
                elif t > man.get('end_time', duration):
                    maneuver_idx += 1
            
            # Kinematic propagation (Euler Step)
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
        """ Returns the state only if the target is alive at this specific time """
        if self.start_time <= t <= self.end_time:
            state = np.zeros(6)
            for i in range(6):
                # Linear interpolation for sub-step precision
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
        """ Returns number of clutter points for current scan based on Poisson distribution """
        return self.rng.poisson(self.clutter_lambda)

    def generate_measurements(self, targets: list, duration: float) -> pd.DataFrame:
        raise NotImplementedError("Subclasses must implement this method")

class RadarSensor(Sensor):
    def generate_measurements(self, targets: list, duration: float) -> pd.DataFrame:
        timestamps = np.arange(0, duration, 1.0 / self.rate_hz)
        data = []
        
        # Extract Bias from configuration (defaults to 0.0 if missing)
        b_r = self.bias.get('range_m', 0.0)
        b_az = np.deg2rad(self.bias.get('azimuth_deg', 0.0))
        b_el = np.deg2rad(self.bias.get('elevation_deg', 0.0))
        
        for t in timestamps:
            # 1. Generate real target measurements
            for target in targets:
                state = target.get_state_at(t)
                if state is None:
                    continue # Target does not exist at this time
                    
                if self.rng.random() < self.drop_prob:
                    continue # Sensor miss (dropout)
                    
                x, y, z, vx, vy, vz = state
                r = np.sqrt(x**2 + y**2 + z**2)
                az = np.arctan2(y, x)
                el = np.arcsin(z / r)
                vr = (x*vx + y*vy + z*vz) / r
                
                # Add Gaussian noise + constant Bias
                meas_r = r + self.rng.normal(0, self.noise['range_m']) + b_r
                meas_az = az + self.rng.normal(0, np.deg2rad(self.noise['azimuth_deg'])) + b_az
                meas_el = el + self.rng.normal(0, np.deg2rad(self.noise['elevation_deg'])) + b_el
                meas_vr = vr + self.rng.normal(0, self.noise['radial_vel_ms'])
                
                data.append({
                    'timestamp': round(t, 3),
                    'sensor': 'RADAR',
                    'target_id': target.id, # Used ONLY for debugging/visualization
                    'range': round(meas_r, 3),
                    'azimuth': round(np.rad2deg(meas_az), 3),
                    'elevation': round(np.rad2deg(meas_el), 3),
                    'radial_velocity': round(meas_vr, 3)
                })
                
            # 2. Generate Clutter (False Alarms) for this scan
            for _ in range(self.get_num_clutter()):
                c_r = self.rng.uniform(1000, 15000) # Spread in a reasonable range
                c_az = self.rng.uniform(0, np.pi)   # Spread in azimuth (0 to 180 deg)
                c_el = self.rng.uniform(0, np.pi/4) # Spread in elevation (0 to 45 deg)
                c_vr = self.rng.uniform(-200, 200)
                
                data.append({
                    'timestamp': round(t, 3),
                    'sensor': 'RADAR',
                    'target_id': 0, # ID 0 represents Clutter / False Target
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
                
            # Generate Clutter for EO
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
# Main Execution & Visualization
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

def visualize_data(truth_df: pd.DataFrame, radar_df: pd.DataFrame, eo_df: pd.DataFrame):
    fig = plt.figure(figsize=(18, 8))
    
    # ---------------------------------------------------------
    # Subplot 1: 3D World View (Truth + Radar)
    # ---------------------------------------------------------
    ax1 = fig.add_subplot(121, projection='3d')
    unique_targets = [tid for tid in truth_df['target_id'].unique() if tid != 0]
    cmap = plt.get_cmap('tab10')
    colors = {tid: cmap(i % 10) for i, tid in enumerate(unique_targets)}
    
    # Plot Ground Truth Trajectories
    for target_id in unique_targets:
        t_df = truth_df[truth_df['target_id'] == target_id]
        ax1.plot(t_df['x'], t_df['y'], t_df['z'], 
                label=f'Target {target_id}', color=colors[target_id], linewidth=2.5)
        ax1.scatter(t_df['x'].iloc[0], t_df['y'].iloc[0], t_df['z'].iloc[0], 
                   color=colors[target_id], marker='o', s=80)
                   
    ax1.scatter(0, 0, 0, color='black', marker='^', s=200, label='Sensor Origin')
    
    if not radar_df.empty:
        # Plot real target radar measurements (Highlighted)
        real_radar = radar_df[radar_df['target_id'] != 0]
        r = real_radar['range'].values
        az = np.deg2rad(real_radar['azimuth'].values)
        el = np.deg2rad(real_radar['elevation'].values)
        radar_x = r * np.cos(el) * np.cos(az)
        radar_y = r * np.cos(el) * np.sin(az)
        radar_z = r * np.sin(el)
        
        ax1.scatter(radar_x, radar_y, radar_z, color='darkorange', edgecolor='black', s=45, alpha=0.8, label='Radar Hits')
        
        # Plot Clutter in faint red crosses
        clutter_radar = radar_df[radar_df['target_id'] == 0]
        if not clutter_radar.empty:
            r = clutter_radar['range'].values
            az = np.deg2rad(clutter_radar['azimuth'].values)
            el = np.deg2rad(clutter_radar['elevation'].values)
            cx = r * np.cos(el) * np.cos(az)
            cy = r * np.cos(el) * np.sin(az)
            cz = r * np.sin(el)
            ax1.scatter(cx, cy, cz, color='red', marker='x', s=20, alpha=0.5, label='Clutter')

    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D World View (Radar + Truth)')
    ax1.legend()

    # ---------------------------------------------------------
    # Subplot 2: 2D Camera Screen View (EO Only)
    # ---------------------------------------------------------
    ax2 = fig.add_subplot(122)
    
    if not eo_df.empty:
        # Plot EO target hits
        for target_id in unique_targets:
            t_eo_df = eo_df[eo_df['target_id'] == target_id]
            ax2.scatter(t_eo_df['azimuth'], t_eo_df['elevation'], 
                        color=colors[target_id], s=20, alpha=0.7, label=f'EO Hits T{target_id}')
        
        # Plot EO Clutter
        clutter_eo = eo_df[eo_df['target_id'] == 0]
        if not clutter_eo.empty:
            ax2.scatter(clutter_eo['azimuth'], clutter_eo['elevation'], 
                        color='red', marker='x', s=15, alpha=0.4, label='EO Clutter')
    
    ax2.set_xlabel('Azimuth [deg]')
    ax2.set_ylabel('Elevation [deg]')
    ax2.set_title('2D Camera View (EO Sensors)')
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig('data/dataset_visualization_combined.png')
    print("[+] Saved combined visualization to data/dataset_visualization_combined.png")

def main():
    print("--- Starting Ultimate Production-Grade Data Generation ---")
    config = load_config()
    rng = np.random.default_rng(config['simulation']['random_seed'])
    duration = config['simulation']['duration_sec']
    
    targets = []
    for t_conf in config['targets']:
        maneuvers = t_conf.get('maneuvers', [])
        # Extract start and end times to handle track initiation/deletion
        start_t = t_conf.get('start_time', 0.0)
        end_t = t_conf.get('end_time', duration)
        targets.append(Target(t_conf['id'], start_t, end_t, t_conf['initial_position'], t_conf['initial_velocity'], maneuvers, duration))
        
    truth_df = generate_truth_data(targets, duration, config['simulation']['truth_rate_hz'])
    truth_df.to_csv('data/ground_truth.csv', index=False)
    print(f"[+] Saved data/ground_truth.csv ({len(truth_df)} rows)")
    
    radar = RadarSensor("RADAR", config['sensors']['radar'], rng)
    radar_df = radar.generate_measurements(targets, duration)
    # Drop target_id to prevent cheating in the C++ tracker algorithm
    radar_export = radar_df.drop(columns=['target_id'])
    radar_export.to_csv('data/radar.csv', index=False)
    print(f"[+] Saved data/radar.csv ({len(radar_export)} rows, including clutter)")
    
    eo = EOSensor("EO", config['sensors']['eo'], rng)
    eo_df = eo.generate_measurements(targets, duration)
    eo_export = eo_df.drop(columns=['target_id'])
    eo_export.to_csv('data/eo.csv', index=False)
    print(f"[+] Saved data/eo.csv ({len(eo_export)} rows, including clutter)")
    
    # Plot using the data that contains IDs for accurate coloring
    visualize_data(truth_df, radar_df, eo_df)
    print("--- Data Generation Complete ---")

if __name__ == "__main__":
    main()