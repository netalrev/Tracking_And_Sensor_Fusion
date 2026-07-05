#include "SensorDataLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cmath>

bool SensorDataLoader::loadDataToQueue(const std::string& radar_file, 
                                       const std::string& eo_file, 
                                       EventQueue& queue) {
    try {
        loadRadarData(radar_file, queue);
        loadEOData(eo_file, queue);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Error] Failed to load sensor data: " << e.what() << std::endl;
        return false;
    }
}

void SensorDataLoader::loadRadarData(const std::string& filepath, EventQueue& queue) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open Radar CSV: " + filepath);
    }

    std::string line;
    // Skip the CSV header line
    std::getline(file, line);

    // Hardcoded Radar Noise Covariance (R) based on configuration (variances = std_dev^2)
    // std_devs: range=15.0m, az=0.5deg, el=0.5deg, vr=2.0m/s
    Eigen::Matrix4d R_radar;
    // התאמה ל-Hard Mode ב-config.yaml
    double az_el_std_rad = 1.2 * M_PI / 180.0; // רעש של 1.2 מעלות
    R_radar << 20.0 * 20.0, 0, 0, 0,           // רעש טווח של 20 מטר
               0, az_el_std_rad * az_el_std_rad, 0, 0,
               0, 0, az_el_std_rad * az_el_std_rad, 0,
               0, 0, 0, 2.5 * 2.5;             // רעש מהירות רדיאלית 2.5

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;

        // CSV Format: timestamp,sensor,range,azimuth,elevation,radial_velocity
        std::getline(ss, cell, ','); double timestamp = std::stod(cell);
        std::getline(ss, cell, ','); // skip 'sensor' column
        std::getline(ss, cell, ','); double range = std::stod(cell);
        std::getline(ss, cell, ','); double az_deg = std::stod(cell);
        std::getline(ss, cell, ','); double el_deg = std::stod(cell);
        std::getline(ss, cell, ','); double vr = std::stod(cell);

        // Convert degrees to radians for the math engine
        double az_rad = az_deg * M_PI / 180.0;
        double el_rad = el_deg * M_PI / 180.0;

        // Construct object dynamically and push to queue, transferring ownership (zero-copy)
        queue.push(std::make_unique<RadarMeasurement>(timestamp, range, az_rad, el_rad, vr, R_radar));
    }
}

void SensorDataLoader::loadEOData(const std::string& filepath, EventQueue& queue) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open EO CSV: " + filepath);
    }

    std::string line;
    // Skip the CSV header line
    std::getline(file, line);

    // Hardcoded EO Noise Covariance (R) (std_devs: az=0.2deg, el=0.2deg)
    Eigen::Matrix2d R_eo;
    // התאמה ל-Hard Mode ב-config.yaml
    double eo_std_rad = 0.15 * M_PI / 180.0;   // רעש מצלמה של 0.15 מעלות
    R_eo << eo_std_rad * eo_std_rad, 0,
            0, eo_std_rad * eo_std_rad;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;

        // CSV Format: timestamp,sensor,azimuth,elevation
        std::getline(ss, cell, ','); double timestamp = std::stod(cell);
        std::getline(ss, cell, ','); // skip 'sensor' column
        std::getline(ss, cell, ','); double az_deg = std::stod(cell);
        std::getline(ss, cell, ','); double el_deg = std::stod(cell);

        double az_rad = az_deg * M_PI / 180.0;
        double el_rad = el_deg * M_PI / 180.0;

        queue.push(std::make_unique<EOMeasurement>(timestamp, az_rad, el_rad, R_eo));
    }
}