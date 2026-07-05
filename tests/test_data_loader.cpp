#include "SensorDataLoader.h"
#include <iostream>
#include <cassert>
#include <string>

int main() {
    std::cout << "[Test] Starting SensorDataLoader Test..." << std::endl;

    // Adjust these paths if you run the executable from a different working directory
    const std::string radar_file = "../data/radar.csv";
    const std::string eo_file = "../data/eo.csv";

    EventQueue queue;

    // 1. Load Data
    bool success = SensorDataLoader::loadDataToQueue(radar_file, eo_file, queue);
    if (!success) {
        std::cerr << "[Test Failed] Could not load CSV files." << std::endl;
        return 1;
    }

    std::cout << "[Test] Successfully loaded CSVs. Queue size: " << queue.size() << std::endl;

    // 2. Verify Monotonic Time (Chronological Order)
    double last_timestamp = -1.0;
    int count = 0;

    while (!queue.empty()) {
        // Access the top element (the one with the smallest timestamp)
        const auto& measurement = queue.top();
        double current_timestamp = measurement->getTimestamp();

        // CRITICAL ARCHITECTURE CHECK: Time must never flow backwards!
        assert(current_timestamp >= last_timestamp && "Fatal Error: Time flowed backwards in the queue!");

        last_timestamp = current_timestamp;
        queue.pop();
        count++;
    }

    std::cout << "[Test Passed] Processed " << count << " measurements in perfect chronological order." << std::endl;
    return 0;
}

// Note: This test assumes that the CSV files are correctly formatted and contain valid data.