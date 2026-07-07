#include "Track.h"
#include "DataAssociation.h"
#include "Measurement.h"
#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <cassert>

int main() {
    std::cout << "=========================================\n";
    std::cout << "   Track & Data Association Unit Test    \n";
    std::cout << "=========================================\n\n";

    // 1. Create an initial Radar measurement to spawn a Track
    // Target is at 1000m on the X-axis, moving at 10m/s radially.
    Eigen::Matrix4d R_radar = Eigen::Matrix4d::Identity();
    R_radar(0,0) = 15.0 * 15.0; // Range variance
    R_radar(1,1) = 0.01;        // Azimuth variance
    R_radar(2,2) = 0.01;        // Elevation variance
    R_radar(3,3) = 2.0 * 2.0;   // Velocity variance

    auto initial_meas = std::make_unique<RadarMeasurement>(0.0, 1000.0, 0.0, 0.0, 10.0, R_radar);
    
    // Create the track list and add our first track (ID = 1)
    std::vector<std::unique_ptr<Track>> tracks;
    tracks.push_back(std::make_unique<Track>(1, initial_meas.get()));

    std::cout << "[Step 1] Track 1 created at t=0.0. State: TENTATIVE.\n";

    // 2. Predict forward by 1 second (Target should move slightly forward)
    tracks[0]->predict(1.0);
    std::cout << "[Step 2] Track 1 predicted to t=1.0.\n";

    // 3. Create a GOOD EO measurement (Azimuth is slightly off 0, matching the prediction uncertainty)
    Eigen::Matrix2d R_eo = Eigen::Matrix2d::Identity() * 0.001;
    auto good_eo_meas = std::make_unique<EOMeasurement>(1.0, 0.001, 0.0, R_eo);

    // 4. Create a BAD EO measurement (Clutter - way off at 90 degrees)
    auto bad_eo_meas = std::make_unique<EOMeasurement>(1.0, M_PI/2.0, 0.0, R_eo);

    std::cout << "[Step 3] Testing Data Association...\n";

    // Test the GOOD measurement
    Track* matched_track_good = DataAssociation::findBestMatch(tracks, good_eo_meas.get());
    if (matched_track_good != nullptr) {
        std::cout << " -> SUCCESS: Good measurement matched with Track ID: " << matched_track_good->getId() << "\n";
    } else {
        std::cout << " -> FAILED: Good measurement was incorrectly rejected!\n";
    }

    // Test the BAD measurement
    Track* matched_track_bad = DataAssociation::findBestMatch(tracks, bad_eo_meas.get());
    if (matched_track_bad == nullptr) {
        std::cout << " -> SUCCESS: Bad measurement (Clutter) was correctly rejected by the Mahalanobis Gate!\n";
    } else {
        std::cout << " -> FAILED: Bad measurement was incorrectly matched to Track ID: " << matched_track_bad->getId() << "\n";
    }

    std::cout << "\n=========================================\n";
    std::cout << " TEST COMPLETE \n";
    std::cout << "=========================================\n";

    return 0;
}