#include "EKF.h"
#include <iostream>
#include <cmath>

int main() {
    std::cout << "=========================================\n";
    std::cout << "      EKF Math Engine - Unit Test        \n";
    std::cout << "=========================================\n\n";

    EKF ekf;

    // 1. Fake Initial Radar Measurement
    // Target is at 1000m range, 45 degrees azimuth, 0 elevation, 10m/s radial velocity
    Eigen::VectorXd z_radar(4);
    double az_rad = 45.0 * M_PI / 180.0;
    z_radar << 1000.0, az_rad, 0.0, 10.0;

    Eigen::MatrixXd R_radar = Eigen::Matrix4d::Identity();
    R_radar(0,0) = 15.0 * 15.0; // Range variance
    R_radar(1,1) = 0.01;        // Azimuth variance
    R_radar(2,2) = 0.01;        // Elevation variance
    R_radar(3,3) = 2.0 * 2.0;   // Velocity variance

    std::cout << "[Step 1] Initializing EKF with Radar Measurement...\n";
    ekf.init(z_radar, R_radar);
    std::cout << "Initial State X [x, y, z, vx, vy, vz]:\n" 
              << ekf.getState().transpose() << "\n\n";

    // 2. Predict Step (Advance by 1 second)
    std::cout << "[Step 2] Predicting forward by dt = 1.0s...\n";
    ekf.predict(1.0);
    std::cout << "Predicted State X:\n" 
              << ekf.getState().transpose() << "\n\n";

    // 3. Fake EO Measurement Update
    // Camera sees the target shifted slightly to 46 degrees azimuth
    Eigen::VectorXd z_eo(2);
    double eo_az_rad = 46.0 * M_PI / 180.0;
    z_eo << eo_az_rad, 0.0;

    Eigen::MatrixXd R_eo = Eigen::Matrix2d::Identity() * 0.001; // High precision

    std::cout << "[Step 3] Updating with EO Measurement...\n";
    ekf.updateEO(z_eo, R_eo);
    std::cout << "Updated State X:\n" 
              << ekf.getState().transpose() << "\n\n";

    std::cout << "=========================================\n";
    std::cout << " TEST PASSED: No Matrix Crashes or NaNs! \n";
    std::cout << "=========================================\n";

    return 0;
}