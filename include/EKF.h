#pragma once

#include <Eigen/Dense>

/**
 * @brief Extended Kalman Filter (EKF) mathematical engine for 3D tracking.
 * Uses a Constant Velocity (CV) model.
 * State Vector X: [x, y, z, vx, vy, vz]^T
 */
class EKF {
public:
    // Eigen fixed-size types for maximum performance (loop unrolling, zero heap allocation)
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

    /**
     * @brief Constructor. Initializes the filter with default process noise.
     */
    EKF();

    /**
     * @brief Initializes the state and covariance from the first Radar measurement.
     * @param z_radar Radar measurement vector [r, az, el, vr]^T
     * @param R_radar Radar measurement noise covariance
     */
    void init(const Eigen::VectorXd& z_radar, const Eigen::MatrixXd& R_radar);

    /**
     * @brief Predicts the state and covariance forward by dt seconds.
     * @param dt Time step in seconds
     */
    void predict(double dt);

    /**
     * @brief Updates the state using a Radar measurement.
     * @param z Radar measurement vector [r, az, el, vr]^T
     * @param R Radar noise covariance matrix (4x4)
     */
    void updateRadar(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);

    /**
     * @brief Updates the state using an EO measurement.
     * @param z EO measurement vector [az, el]^T
     * @param R EO noise covariance matrix (2x2)
     */
    void updateEO(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);

    // Getters for external logic (like Data Association)
    [[nodiscard]] Vector6d getState() const { return x_; }
    [[nodiscard]] Matrix6d getCovariance() const { return P_; }

    // Mahalanobis distance calculations for Data Association
    [[nodiscard]] double computeMahalanobisRadar(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;
    [[nodiscard]] double computeMahalanobisEO(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;

private:
    Vector6d x_; // State vector: [x, y, z, vx, vy, vz]^T
    Matrix6d P_; // State covariance matrix

    // Process noise variance (acceleration variance)
    double process_noise_variance_;

    // Private helper functions to compute non-linear Jacobians
    [[nodiscard]] Eigen::Matrix<double, 4, 6> computeRadarJacobian(const Vector6d& state) const;
    [[nodiscard]] Eigen::Matrix<double, 2, 6> computeEOJacobian(const Vector6d& state) const;
};