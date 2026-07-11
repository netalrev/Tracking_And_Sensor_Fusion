#include "Track.h"
#include <cassert>

Track::Track(int id, const Measurement* initial_measurement)
    : track_id_(id),
      state_(TrackState::TENTATIVE),
      time_last_updated_(initial_measurement->getTimestamp()),
      time_last_measurement_(initial_measurement->getTimestamp()),
      last_radar_time_(initial_measurement->getTimestamp()),
      last_eo_time_(-1.0),
      hit_streak_(1),
      missed_updates_(0) {
    
    // CRITICAL ARCHITECTURE CHECK: 
    // We cannot initialize a 3D track from a 2D EO measurement.
    // In production, an assertion here catches logical bugs from the TrackerManager.
    assert(initial_measurement->getType() == SensorType::RADAR && "Track must be initialized with Radar!");
    
    // Initialize the EKF with the first measurement
    ekf_.init(initial_measurement->getVector(), initial_measurement->getCovariance());
}

void Track::predict(double current_time) {
    double dt = current_time - time_last_updated_;
    
    // Zero-dt check: If two sensors fire at the exact same millisecond, 
    // we don't predict (since time hasn't moved). We just fuse the updates sequentially.
    if (dt > 0.0) {
        ekf_.predict(dt);
        time_last_updated_ = current_time;
    }
}

void Track::update(const Measurement* measurement) {
    if (measurement->getType() == SensorType::RADAR) {
        ekf_.updateRadar(measurement->getVector(), measurement->getCovariance());
        
        last_radar_time_ = measurement->getTimestamp();
        hit_streak_++; 
        
        // המכ"ם תמיד מאפס את שעון המוות ומאריך את חיי המטרה
        missed_updates_ = 0;
        time_last_measurement_ = measurement->getTimestamp(); 

    } else if (measurement->getType() == SensorType::EO) {
        ekf_.updateEO(measurement->getVector(), measurement->getCovariance());
        
        last_eo_time_ = measurement->getTimestamp();
    }

    if (state_ == TrackState::TENTATIVE && hit_streak_ >= HITS_TO_CONFIRM) {
        state_ = TrackState::CONFIRMED;
    }
}

double Track::getMahalanobisDistance(const Measurement* measurement) const {
    if (measurement->getType() == SensorType::RADAR) {
        return ekf_.computeMahalanobisRadar(measurement->getVector(), measurement->getCovariance());
    } else {
        return ekf_.computeMahalanobisEO(measurement->getVector(), measurement->getCovariance());
    }
}