#pragma once

#include <Eigen/Dense>

/**
 * @brief Identifies the source of the measurement.
 */
enum class SensorType {
    RADAR,
    EO
};

/**
 * @brief Abstract Base Class for all sensor measurements.
 * Defines the polymorphic interface required by the Event Queue and the Tracker.
 */
class Measurement {
public:
    // Virtual destructor is critical for polymorphic base classes to prevent memory leaks
    virtual ~Measurement() = default;

    // Core getters that every measurement must implement
    [[nodiscard]] virtual double getTimestamp() const = 0;
    [[nodiscard]] virtual SensorType getType() const = 0;

    /**
     * @brief Returns the raw measurement vector (Z).
     * Using dynamic size (VectorXd) in the interface allows the EKF to process 
     * both 4D Radar and 2D EO vectors polymorphically.
     */
    [[nodiscard]] virtual Eigen::VectorXd getVector() const = 0;

    /**
     * @brief Returns the measurement noise covariance matrix (R).
     */
    [[nodiscard]] virtual Eigen::MatrixXd getCovariance() const = 0;
};

/**
 * @brief Represents a 4D Radar Measurement: [Range, Azimuth, Elevation, Radial Velocity]
 */
class RadarMeasurement : public Measurement {
private:
    double timestamp_;
    Eigen::Vector4d z_; 
    Eigen::Matrix4d R_; 

public:
    RadarMeasurement(double timestamp, 
                     double range, 
                     double azimuth, 
                     double elevation, 
                     double radial_velocity, 
                     const Eigen::Matrix4d& R)
        : timestamp_(timestamp), R_(R) {
        z_ << range, azimuth, elevation, radial_velocity;
    }

    [[nodiscard]] double getTimestamp() const override { return timestamp_; }
    [[nodiscard]] SensorType getType() const override { return SensorType::RADAR; }
    
    // Implicit conversion from fixed size (Vector4d/Matrix4d) to dynamic size (VectorXd/MatrixXd)
    [[nodiscard]] Eigen::VectorXd getVector() const override { return z_; }
    [[nodiscard]] Eigen::MatrixXd getCovariance() const override { return R_; }
};

/**
 * @brief Represents a 2D EO (Electro-Optical) Measurement: [Azimuth, Elevation]
 */
class EOMeasurement : public Measurement {
private:
    double timestamp_;
    Eigen::Vector2d z_;
    Eigen::Matrix2d R_;

public:
    EOMeasurement(double timestamp, 
                  double azimuth, 
                  double elevation, 
                  const Eigen::Matrix2d& R)
        : timestamp_(timestamp), R_(R) {
        z_ << azimuth, elevation;
    }

    [[nodiscard]] double getTimestamp() const override { return timestamp_; }
    [[nodiscard]] SensorType getType() const override { return SensorType::EO; }

    [[nodiscard]] Eigen::VectorXd getVector() const override { return z_; }
    [[nodiscard]] Eigen::MatrixXd getCovariance() const override { return R_; }
};