#include "TrackerManager.h"
#include <iostream>
#include <string>
#include <exception>

int main() {
    std::cout << "=========================================\n";
    std::cout << "  Multi-Sensor Tracking & Fusion System  \n";
    std::cout << "=========================================\n\n";

    try {
        // Define file paths. 
        // Assuming the executable is run from the 'build' directory.
        const std::string radar_file = "../data/radar.csv";
        const std::string eo_file = "../data/eo.csv";
        const std::string output_file = "../data/fused_tracks.csv";

        // Initialize the Tracker Manager
        TrackerManager tracker(radar_file, eo_file, output_file);

        // Execute the event-driven tracking loop
        tracker.run();

    } catch (const std::exception& e) {
        // Top-level exception handling prevents silent crashes
        std::cerr << "\n[FATAL ERROR] The system crashed with exception: \n" 
                  << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "\n[FATAL ERROR] An unknown error occurred!" << std::endl;
        return 1;
    }

    std::cout << "\n[SUCCESS] System gracefully terminated.\n";
    return 0;
}
