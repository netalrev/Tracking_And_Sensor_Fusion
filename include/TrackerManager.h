#pragma once

#include "SensorDataLoader.h"
#include "Track.h"
#include <vector>
#include <memory>
#include <string>

/**
 * @class TrackerManager
 * @brief Central orchestrator of the Tracking System.
 * 
 * Consumes the priority queue and executes the tracking loop for all targets.
 */
class TrackerManager {
public:
    /**
     * @brief Constructor for the TrackerManager.
     * @param radar_csv Path to the input radar data file.
     * @param eo_csv Path to the input EO data file.
     * @param output_csv Path to save the fused output tracks.
     * @param export_tentative Toggle to export unconfirmed tracks and status column (Default: true).
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
    EventQueue event_queue_;                           ///< Queue of sensor measurements sorted by timestamp.
    std::vector<std::unique_ptr<Track>> active_tracks_; ///< Container mapping active track IDs to models.
    
    int next_track_id_;                                ///< Counter for assigning unique IDs to new tracks.
    std::string output_filepath_;                      ///< Filepath for the resulting track CSV output.

    bool export_tentative_;                            ///< Internal flag to determine if unconfirmed tracks are exported.

    /**
     * @brief Advances all active tracks to the current measurement time.
     * @param current_time Time to synchronize the filters to in seconds.
     */
    void predictAll(double current_time);

    /**
     * @brief Handles track initiation for unassociated measurements.
     * @param measurement Pointer to the non-associated measurement data.
     */
    void handleUnassociatedMeasurement(const Measurement* measurement);

    /**
     * @brief Removes tracks that have transitioned to the DEAD state.
     * @param current_time The current operational time (for logging/debugging).
     */
    void cleanupDeadTracks(double current_time);

    /**
     * @brief Exports the current state of tracks to the output CSV.
     * @param current_time The timestamp corresponding to the exported states.
     * @param out_file Reference to the active filestream.
     */
    void exportTrackStates(double current_time, std::ofstream& out_file);
};