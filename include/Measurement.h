#pragma once

#include <Eigen/Dense>

/**
 * @enum SensorType
 * @brief Identifies the sensor source of the measurement.
 */
enum class SensorType {
    RADAR,
    EO
};

/**
 * @class Measurement
 * @brief Abstract Base Class for all sensor measurements.
 * 
 * Defines the polymorphic interface required by the Event Queue and the Tracking Engine.
 */
class Measurement {
public:
    /**
     * @brief Virtual destructor to ensure safe polymorphic destruction.
     */
    virtual ~Measurement() = default;

    /**
     * @brief Retrieves the measurement timestamp.
     * @return The time of the measurement in seconds.
     */
    [[nodiscard]] virtual double getTimestamp() const = 0;

    /**
     * @brief Retrieves the sensor type generating this measurement.
     * @return SensorType enumerator (e.g., RADAR or EO).
     */
    [[nodiscard]] virtual SensorType getType() const = 0;

    /**
     * @brief Retrieves the raw measurement vector (Z).
     * @return Dynamic size vector capturing the measurement state.
     */
    [[nodiscard]] virtual Eigen::VectorXd getVector() const = 0;

    /**
     * @brief Retrieves the measurement noise covariance matrix (R).
     * @return Dynamic size matrix representing measurement uncertainty.
     */
    [[nodiscard]] virtual Eigen::MatrixXd getCovariance() const = 0;
};

/**
 * @class RadarMeasurement
 * @brief Represents a 4D Radar Measurement: [Range, Azimuth, Elevation, Radial Velocity].
 */
class RadarMeasurement : public Measurement {
private:
    double timestamp_;        ///< Time of the radar measurement.
    Eigen::Vector4d z_;       ///< 4D measurement vector.
    Eigen::Matrix4d R_;       ///< 4x4 measurement noise covariance matrix.

public:
    /**
     * @brief Constructor for RadarMeasurement.
     * @param timestamp Time of the measurement in seconds.
     * @param range Range in meters.
     * @param azimuth Azimuth in radians.
     * @param elevation Elevation in radians.
     * @param radial_velocity Radial velocity in meters per second.
     * @param R 4x4 Noise covariance matrix.
     */
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
    
    [[nodiscard]] Eigen::VectorXd getVector() const override { return z_; }
    [[nodiscard]] Eigen::MatrixXd getCovariance() const override { return R_; }
};

/**
 * @class EOMeasurement
 * @brief Represents a 2D Electro-Optical (EO) Measurement: [Azimuth, Elevation].
 */
class EOMeasurement : public Measurement {
private:
    double timestamp_;        ///< Time of the EO measurement.
    Eigen::Vector2d z_;       ///< 2D measurement vector.
    Eigen::Matrix2d R_;       ///< 2x2 measurement noise covariance matrix.

public:
    /**
     * @brief Constructor for EOMeasurement.
     * @param timestamp Time of the measurement in seconds.
     * @param azimuth Azimuth in radians.
     * @param elevation Elevation in radians.
     * @param R 2x2 Noise covariance matrix.
     */
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