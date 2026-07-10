#pragma once

#include "Measurement.h"
#include <queue>
#include <memory>
#include <string>
#include <vector>

/**
 * @struct MeasurementCompare
 * @brief Custom comparator to turn std::priority_queue into a Min-Heap.
 * 
 * Ensures the measurement with the SMALLEST timestamp surfaces to the top.
 */
struct MeasurementCompare {
    bool operator()(const std::unique_ptr<Measurement>& a, const std::unique_ptr<Measurement>& b) const {
        return a->getTimestamp() > b->getTimestamp();
    }
};

/**
 * @typedef EventQueue
 * @brief Type alias for the Event Queue. 
 * 
 * Holds unique pointers to polymorhic Measurements, sorted by the custom comparator.
 */
using EventQueue = std::priority_queue<std::unique_ptr<Measurement>, 
                                       std::vector<std::unique_ptr<Measurement>>, 
                                       MeasurementCompare>;

/**
 * @class SensorDataLoader
 * @brief Static utility class to parse CSVs and load them into the Event Queue.
 */
class SensorDataLoader {
public:
    /**
     * @brief Parses both Radar and EO CSV files and populates the queue.
     * @param radar_file Path to the input radar CSV file.
     * @param eo_file Path to the input EO CSV file.
     * @param queue The priority queue to populate (passed by reference).
     * @return True if successful, false otherwise.
     */
    static bool loadDataToQueue(const std::string& radar_file, 
                                const std::string& eo_file, 
                                EventQueue& queue);

private:
    /**
     * @brief Internal helper to parse Radar data.
     * @param filepath Path to the input radar CSV file.
     * @param queue The priority queue to populate.
     */
    static void loadRadarData(const std::string& filepath, EventQueue& queue);

    /**
     * @brief Internal helper to parse EO data.
     * @param filepath Path to the input EO CSV file.
     * @param queue The priority queue to populate.
     */
    static void loadEOData(const std::string& filepath, EventQueue& queue);
};