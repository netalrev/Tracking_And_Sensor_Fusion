#include "TrackerManager.h"
#include "DataAssociation.h"
#include <iostream>
#include <fstream>
#include <algorithm>

TrackerManager::TrackerManager(const std::string& radar_csv, 
                               const std::string& eo_csv, 
                               const std::string& output_csv,
                               bool export_tentative) 
    : next_track_id_(1), output_filepath_(output_csv),export_tentative_(export_tentative) {
    
    std::cout << "[TrackerManager] Loading data from sensors...\n";
    SensorDataLoader::loadDataToQueue(radar_csv, eo_csv, event_queue_);
    std::cout << "[TrackerManager] Data loaded. Queue size: " << event_queue_.size() << "\n";
}

void TrackerManager::run() {
    std::cout << "[TrackerManager] Starting main tracking loop...\n";
    
    // Open CSV file and write the header based on the toggle
    std::ofstream out_file(output_filepath_);
    if (export_tentative_) {
        out_file << "timestamp,target_id,x,y,z,vx,vy,vz,status\n"; 
    } else {
        out_file << "timestamp,target_id,x,y,z,vx,vy,vz\n"; 
    }

    double current_time = 0.0;
    double last_export_time = -1.0;

    while (!event_queue_.empty()) {
        // 1. Peek at the top measurement (Zero-Copy: We just get a raw pointer)
        const Measurement* measurement = event_queue_.top().get();
        current_time = measurement->getTimestamp();

        // 2. Predict step: Advance all existing tracks to the new event's time
        predictAll(current_time);

        // 3. Data Association step: Find the most statistically probable track
        Track* matched_track = DataAssociation::findBestMatch(active_tracks_, measurement);

        // 4. Update or Initiate step
        if (matched_track != nullptr) {
            matched_track->update(measurement);
        } else {
            handleUnassociatedMeasurement(measurement);
        }

        // 5. Memory Management: Pop destroys the unique_ptr and frees the measurement from RAM instantly
        event_queue_.pop();

        // 6. Track Management: Remove tracks that starved or died
        cleanupDeadTracks(current_time);

        // 7. Export results at 10Hz intervals (every 0.1 sec)
        // This prevents creating a massive CSV file with duplicated timestamps
        if (current_time - last_export_time >= 0.1) {
            exportTrackStates(current_time, out_file);
            last_export_time = current_time;
        }
    }
    
    out_file.close();
    std::cout << "[TrackerManager] Tracking complete! Results saved to " << output_filepath_ << "\n";
}

void TrackerManager::predictAll(double current_time) {
    for (auto& track : active_tracks_) {
        track->predict(current_time);
    }
}

void TrackerManager::handleUnassociatedMeasurement(const Measurement* measurement) {
    // Only Radar can initiate tracks (we need Range to get a 3D Cartesian position)
    if (measurement->getType() == SensorType::RADAR) {
        active_tracks_.push_back(std::make_unique<Track>(next_track_id_++, measurement));
    }
}

void TrackerManager::cleanupDeadTracks(double current_time) {
    // Erase-Remove Idiom: Cleanly remove tracks from the vector
    active_tracks_.erase(
        std::remove_if(active_tracks_.begin(), active_tracks_.end(),
            [current_time](const std::unique_ptr<Track>& t) { 
                // A track is removed if it was explicitly marked DEAD,
                // OR if it starved (hasn't received ANY measurement in the last 5.0 seconds)
                bool is_dead = (t->getState() == TrackState::DEAD);
                bool is_starved = (current_time - t->getLastMeasurementTime() > 5.0);
                return is_dead || is_starved; 
            }),
        active_tracks_.end()
    );
}

void TrackerManager::exportTrackStates(double current_time, std::ofstream& out_file) {
    for (const auto& track : active_tracks_) {
        
        if (export_tentative_) {
            // New Mode: Export everything that is alive, plus the status column
            if (track->getState() != TrackState::DEAD) {
                EKF::Vector6d state = track->getKinematicState();
                std::string status_str = (track->getState() == TrackState::CONFIRMED) ? "CONFIRMED" : "TENTATIVE";
                
                out_file << current_time << ","
                         << track->getId() << ","
                         << state(0) << "," << state(1) << "," << state(2) << ","
                         << state(3) << "," << state(4) << "," << state(5) << ","
                         << status_str << "\n";
            }
        } else {
            // Classic Mode: Export ONLY CONFIRMED tracks, match ground_truth format exactly
            if (track->getState() == TrackState::CONFIRMED) {
                EKF::Vector6d state = track->getKinematicState();
                
                out_file << current_time << ","
                         << track->getId() << ","
                         << state(0) << "," << state(1) << "," << state(2) << ","
                         << state(3) << "," << state(4) << "," << state(5) << "\n";
            }
        }
    }
}