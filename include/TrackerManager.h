#pragma once

#include "SensorDataLoader.h"
#include "Track.h"
#include <vector>
#include <memory>
#include <string>

/**
 * @brief The central orchestrator of the Tracking System.
 * Consumes the priority queue and executes the tracking loop for all targets.
 */
class TrackerManager {
public:
    /**
     * @brief Constructor.
     * @param radar_csv Path to radar data
     * @param eo_csv Path to eo data
     * @param output_csv Path to save the fused output tracks
     * @param export_tentative Toggle to export unconfirmed tracks and status column (Default: true)
     */
    TrackerManager(const std::string& radar_csv, 
                   const std::string& eo_csv, 
                   const std::string& output_csv,
                   bool export_tentative = true);

    /**
     * @brief The main execution function. 
     * Runs the event-driven tracking loop until the queue is empty.
     */
    void run();

private:
    EventQueue event_queue_;
    std::vector<std::unique_ptr<Track>> active_tracks_;
    
    int next_track_id_;
    std::string output_filepath_;

    bool export_tentative_;

    // ==========================================
    // Internal Tracking Loop Steps
    // ==========================================
    
    /**
     * @brief Advances all active tracks to the current measurement time.
     */
    void predictAll(double current_time);

    /**
     * @brief Handles track initiation for unassociated measurements.
     */
    void handleUnassociatedMeasurement(const Measurement* measurement);

    /**
     * @brief Removes tracks that have transitioned to the DEAD state.
     */
    void cleanupDeadTracks(double current_time);

    /**
     * @brief Exports the current state of CONFIRMED tracks to the output CSV.
     */
    void exportTrackStates(double current_time, std::ofstream& out_file);
};