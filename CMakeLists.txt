cmake_minimum_required(VERSION 3.14)
project(SensorFusionTracking CXX)

# Require C++17 standard for smart pointers and modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3 (Ensure it is installed on your system)
# If Eigen3 is not in a standard path, you may need to specify its path during cmake configuration
find_package(Eigen3 REQUIRED NO_MODULE)

# Define Include Directories (where the compiler looks for .h files)
include_directories(include)

# ==========================================
# Target: test_data_loader
# ==========================================
# This compiles our standalone test executable
add_executable(test_data_loader 
    src/SensorDataLoader.cpp 
    tests/test_data_loader.cpp
)

# Link Eigen3 to the test executable
target_link_libraries(test_data_loader Eigen3::Eigen)

# ==========================================
# Target: test_ekf_math
# ==========================================
add_executable(test_ekf_math 
    src/EKF.cpp 
    tests/test_ekf_math.cpp
)
target_link_libraries(test_ekf_math Eigen3::Eigen)

# ==========================================
# Target: test_track_association
# ==========================================
add_executable(test_track_association 
    src/EKF.cpp 
    src/Track.cpp 
    src/DataAssociation.cpp 
    tests/test_track_association.cpp
)
target_link_libraries(test_track_association Eigen3::Eigen)

# ==========================================
# Target: tracking_system (The Main Application)
# ==========================================
add_executable(tracking_system 
    src/main.cpp
    src/SensorDataLoader.cpp
    src/EKF.cpp
    src/Track.cpp
    src/DataAssociation.cpp
    src/TrackerManager.cpp
)
target_link_libraries(tracking_system Eigen3::Eigen)