#pragma once

#include "Track.h"
#include "Measurement.h"
#include <vector>
#include <memory>

/**
 * @class DataAssociation
 * @brief Handles logic to associate incoming measurements to existing tracks.
 */
class DataAssociation {
public:
    /// Statistical Gating Threshold for Radar (Chi-Square, 99% confidence, 4 DoF)
    static constexpr double GATING_THRESHOLD_RADAR = 13.277; 
    
    /// Statistical Gating Threshold for EO (Chi-Square, 99% confidence, 2 DoF)
    static constexpr double GATING_THRESHOLD_EO = 9.210;

    /**
     * @brief Finds the best existing track for a given measurement using Global Nearest Neighbor (GNN).
     * @param tracks List of active tracks in the system.
     * @param measurement The new measurement to associate.
     * @return Pointer to the matched track, or nullptr if no track passed the gate.
     */
    static Track* findBestMatch(const std::vector<std::unique_ptr<Track>>& tracks, 
                                const Measurement* measurement);
};