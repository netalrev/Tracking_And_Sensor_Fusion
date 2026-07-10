#include "DataAssociation.h"
#include <limits>

Track* DataAssociation::findBestMatch(const std::vector<std::unique_ptr<Track>>& tracks, 
                                      const Measurement* measurement) {
    Track* best_track = nullptr;
    double min_distance = std::numeric_limits<double>::max();
    
    // Strict sensor gating thresholds per degrees of freedom.
    // Radar threshold corresponding to 4 DoF.
    // EO threshold corresponding to 2 DoF.
    double gate_threshold = (measurement->getType() == SensorType::RADAR) ? 
                            GATING_THRESHOLD_RADAR : GATING_THRESHOLD_EO;

    for (const auto& track : tracks) {
        if (track->getState() == TrackState::DEAD) continue;

        // Skip if target has already consumed a hit from this sensor in current timeframe
        if (track->hasProcessedInScan(measurement)) continue;

        double dist_squared = track->getMahalanobisDistance(measurement);

        if (dist_squared < gate_threshold && dist_squared < min_distance) {
            min_distance = dist_squared;
            best_track = track.get();
        }
    }
    return best_track;
}