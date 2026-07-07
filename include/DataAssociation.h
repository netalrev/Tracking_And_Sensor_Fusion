#pragma once

#include "Track.h"
#include "Measurement.h"
#include <vector>
#include <memory>

class DataAssociation {
public:
    // Statistical Gating Thresholds based on Chi-Square distribution (99% confidence)
    // Radar has 4 Degrees of Freedom (Range, Azimuth, Elevation, Vr)
    static constexpr double GATING_THRESHOLD_RADAR = 13.277; 
    // EO has 2 Degrees of Freedom (Azimuth, Elevation)
    static constexpr double GATING_THRESHOLD_EO = 9.210;

    /**
     * @brief Finds the best existing track for a given measurement using Global Nearest Neighbor (GNN).
     * @param tracks List of active tracks in the system.
     * @param measurement The new measurement to associate.
     * @return Pointer to the matched track, or nullptr if no track passed the statistical gate.
     */
    static Track* findBestMatch(const std::vector<std::unique_ptr<Track>>& tracks, 
                                const Measurement* measurement);
};