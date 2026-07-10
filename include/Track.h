#pragma once

#include "EKF.h"
#include "Measurement.h"
#include <memory>

/**
 * @brief Represents the lifecycle state of a tracked target.
 */
enum class TrackState {
    TENTATIVE, // Newly created track from a single Radar hit. Not yet trusted.
    CONFIRMED, // Track has received enough consistent hits to be published.
    DEAD       // Track has missed too many updates and should be deleted.
};

/**
 * @brief Represents a single real-world target. 
 * Manages its own EKF mathematical engine and its lifecycle.
 */
class Track {
public:
    /**
     * @brief Constructor. Initializes a new track.
     * @param id Unique identifier assigned by the Tracker Manager.
     * @param initial_measurement Must be a Radar measurement to provide 3D position!
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
     * @brief Called when a prediction cycle happens but NO measurement was matched.
     * Increases the coasting (missed) counter and may kill the track.
     */
    void markMissed();

    [[nodiscard]] double getMahalanobisDistance(const Measurement* measurement) const;
    // ==========================================
    // Getters
    // ==========================================
    [[nodiscard]] int getId() const { return track_id_; }
    [[nodiscard]] TrackState getState() const { return state_; }
    [[nodiscard]] double getLastUpdateTime() const { return time_last_updated_; }
    
    // For evaluating tracks against ground truth
    [[nodiscard]] EKF::Vector6d getKinematicState() const { return ekf_.getState(); }
    [[nodiscard]] EKF::Matrix6d getCovariance() const { return ekf_.getCovariance(); }

    // Expose the EKF read-only for Data Association (Mahalanobis Distance)
    [[nodiscard]] const EKF& getEKF() const { return ekf_; }
    [[nodiscard]] double getLastMeasurementTime() const { return time_last_measurement_; }

    // ==========================================
    // Scan Mutex: Prevents a track from consuming multiple hits from the same sensor in the same scan.
    // ==========================================
    bool hasProcessedInScan(const Measurement* m) const {
        double last_time = (m->getType() == SensorType::RADAR) ? last_radar_time_ : last_eo_time_;
        return std::abs(m->getTimestamp() - last_time) < 0.001; // True if already updated in this exact timestamp
    }

private:
    int track_id_;
    TrackState state_;
    EKF ekf_; // Composition: A Track *has an* EKF.
    
    double time_last_updated_;
    
    // M/N Logic counters (Track Management)
    int hit_streak_;
    int missed_updates_;

    // Configuration Thresholds for Lifecycle
    // In a full system, these would be loaded from config.yaml
    static constexpr int HITS_TO_CONFIRM = 5;
    static constexpr int MISSES_TO_DEAD = 5;

    double time_last_measurement_; // Timestamp of the last measurement (Radar or EO) that updated this track
    double last_radar_time_; // Timestamp of the last Radar measurement that updated this track
    double last_eo_time_; // Timestamp of the last EO measurement that updated this track
};