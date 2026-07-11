#pragma once

#include "EKF.h"
#include "Measurement.h"
#include <memory>
#include <cmath>

/**
 * @enum TrackState
 * @brief Represents the lifecycle state of a tracked target.
 */
enum class TrackState {
    TENTATIVE, ///< Newly created track from a single Radar hit. Not yet trusted.
    CONFIRMED, ///< Track has received enough consistent hits to be published.
    DEAD       ///< Track has missed too many updates and should be deleted.
};

/**
 * @class Track
 * @brief Represents a single real-world target. 
 * 
 * Manages its own Extended Kalman Filter (EKF) mathematical engine and its lifecycle status.
 */
class Track {
public:
    /**
     * @brief Constructor for Track.
     * @param id Unique identifier assigned by the Tracker Manager.
     * @param initial_measurement Must be a Radar measurement to provide 3D position state initialization.
     */
    Track(int id, const Measurement* initial_measurement);

    /**
     * @brief Advances the track's EKF state to the current time.
     * @param current_time The timestamp of the new measurement being processed.
     */
    void predict(double current_time);

    /**
     * @brief Updates the track's EKF with a new matched measurement.
     * @param measurement Pointer to the matched Radar or EO measurement.
     */
    void update(const Measurement* measurement);

    /**
     * @brief Computes the Mahalanobis distance metric against a candidate measurement.
     * @param measurement The candidate measurement data for correlation testing.
     * @return The scalar Mahalanobis distance based on target state and measurement noise.
     */
    [[nodiscard]] double getMahalanobisDistance(const Measurement* measurement) const;

    // ==========================================
    // Track Accessors
    // ==========================================
    [[nodiscard]] int getId() const { return track_id_; }
    [[nodiscard]] TrackState getState() const { return state_; }
    [[nodiscard]] EKF::Vector6d getKinematicState() const { return ekf_.getState(); }
    [[nodiscard]] double getLastMeasurementTime() const { return time_last_measurement_; }

    /**
     * @brief Scan Mutex: Prevents a track from consuming multiple hits from the same sensor in the exact same scan.
     * @param m The candidate measurement.
     * @return True if the track was already updated by this sensor at this exact timestamp.
     */
    [[nodiscard]] bool hasProcessedInScan(const Measurement* m) const {
        double last_time = (m->getType() == SensorType::RADAR) ? last_radar_time_ : last_eo_time_;
        return std::abs(m->getTimestamp() - last_time) < 0.001; 
    }

private:
    int track_id_;                           ///< Unique identifier across the tracking architecture.
    TrackState state_;                       ///< Current operational lifecycle status.
    EKF ekf_;                                ///< Dedicated Extended Kalman Filter for this specific target.
    
    double time_last_updated_;               ///< Internal timestamp of the last complete update loop.
    
    int hit_streak_;                         ///< Current consecutive measurement hits.
    int missed_updates_;                     ///< Tracker prediction cycles running without measurement data.

    static constexpr int HITS_TO_CONFIRM = 5;///< Configured threshold of hits required for CONFIRMED state.

    double time_last_measurement_;           ///< Timestamp of the last measurement (Radar or EO) that updated this track.
    double last_radar_time_;                 ///< Timestamp of the last Radar measurement that updated this track.
    double last_eo_time_;                    ///< Timestamp of the last EO measurement that updated this track.
};