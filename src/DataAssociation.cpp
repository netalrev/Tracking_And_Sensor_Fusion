#include "DataAssociation.h"
#include <limits>

Track* DataAssociation::findBestMatch(const std::vector<std::unique_ptr<Track>>& tracks, 
                                      const Measurement* measurement) {
    Track* best_track = nullptr;
    double min_distance = std::numeric_limits<double>::max();
    
    // =========================================================
    // שימוש בקבועים המדויקים מ-DataAssociation.h
    // מכ"ם מקבל 13.277 (עבור 4 דרגות חופש), מצלמה מקבלת 9.21
    // =========================================================
    double gate_threshold = (measurement->getType() == SensorType::RADAR) ? 
                            GATING_THRESHOLD_RADAR : GATING_THRESHOLD_EO;

    for (const auto& track : tracks) {
        if (track->getState() == TrackState::DEAD) continue;

        // =========================================================
        // SCAN MUTEX: 1-to-1 Sensor Constraint.
        // A track cannot consume multiple hits from the same sensor at the same time.
        // =========================================================
        if (track->hasProcessedInScan(measurement)) continue;

        double dist_squared = track->getMahalanobisDistance(measurement);

        if (dist_squared < gate_threshold && dist_squared < min_distance) {
            min_distance = dist_squared;
            best_track = track.get();
        }
    }
    return best_track;
}