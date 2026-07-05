#pragma once

#include "Measurement.h"
#include <queue>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Custom comparator to turn std::priority_queue into a Min-Heap.
 * We want the measurement with the SMALLEST timestamp to be at the top.
 */
struct MeasurementCompare {
    bool operator()(const std::unique_ptr<Measurement>& a, const std::unique_ptr<Measurement>& b) const {
        return a->getTimestamp() > b->getTimestamp();
    }
};

/**
 * @brief Type alias for the Event Queue. 
 * Holds unique pointers to polymorhic Measurements, sorted by the custom comparator.
 */
using EventQueue = std::priority_queue<std::unique_ptr<Measurement>, 
                                       std::vector<std::unique_ptr<Measurement>>, 
                                       MeasurementCompare>;

/**
 * @brief Static utility class to parse CSVs and load them into the Event Queue.
 */
class SensorDataLoader {
public:
    /**
     * @brief Parses both Radar and EO CSV files and populates the queue.
     * @param radar_file Path to radar.csv
     * @param eo_file Path to eo.csv
     * @param queue The priority queue to populate (passed by reference)
     * @return True if successful, false otherwise.
     */
    static bool loadDataToQueue(const std::string& radar_file, 
                                const std::string& eo_file, 
                                EventQueue& queue);

private:
    static void loadRadarData(const std::string& filepath, EventQueue& queue);
    static void loadEOData(const std::string& filepath, EventQueue& queue);
};