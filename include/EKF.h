#pragma once

#include <Eigen/Dense>

/**
 * @class EKF
 * @brief Extended Kalman Filter mathematical engine for 3D tracking.
 * 
 * Implements a Constant Velocity (CV) model.
 * State Vector X: [x, y, z, vx, vy, vz]^T
 */
class EKF {
public:
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

    /**
     * @brief Default constructor. Initializes the filter with process noise variance.
     */
    EKF();

    /**
     * @brief Initializes the internal state and covariance from the first Radar measurement.
     * @param z_radar Radar measurement vector [r, az, el, vr]^T.
     * @param R_radar Radar measurement noise covariance matrix.
     */
    void init(const Eigen::VectorXd& z_radar, const Eigen::MatrixXd& R_radar);

    /**
     * @brief Predicts the system state and covariance forward in time.
     * @param dt Time step in seconds.
     */
    void predict(double dt);

    /**
     * @brief Updates the system state using a Radar measurement.
     * @param z Radar measurement vector [r, az, el, vr]^T.
     * @param R Radar measurement noise covariance matrix (4x4).
     */
    void updateRadar(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);

    /**
     * @brief Updates the system state using an Electro-Optical (EO) measurement.
     * @param z EO measurement vector [az, el]^T.
     * @param R EO measurement noise covariance matrix (2x2).
     */
    void updateEO(const Eigen::VectorXd& z, const Eigen::MatrixXd& R);

    /**
     * @brief Retrieves the current state vector.
     * @return 6D State vector [x, y, z, vx, vy, vz]^T.
     */
    [[nodiscard]] Vector6d getState() const { return x_; }

    /**
     * @brief Retrieves the current state covariance matrix.
     * @return 6x6 Covariance matrix.
     */
    [[nodiscard]] Matrix6d getCovariance() const { return P_; }

    /**
     * @brief Computes Mahalanobis distance for a Radar measurement against the current state.
     * @param z Radar measurement vector.
     * @param R Radar measurement covariance matrix.
     * @return Mahalanobis distance scalar.
     */
    [[nodiscard]] double computeMahalanobisRadar(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;

    /**
     * @brief Computes Mahalanobis distance for an EO measurement against the current state.
     * @param z EO measurement vector.
     * @param R EO measurement covariance matrix.
     * @return Mahalanobis distance scalar.
     */
    [[nodiscard]] double computeMahalanobisEO(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const;

private:
    Vector6d x_; ///< State vector: [x, y, z, vx, vy, vz]^T
    Matrix6d P_; ///< State covariance matrix

    double process_noise_variance_; ///< System process noise variance (acceleration variance)

    /**
     * @brief Computes the Jacobian of the measurement model for Radar.
     * @param state The current system state [x, y, z, vx, vy, vz]^T.
     * @return 4x6 Jacobian matrix.
     */
    [[nodiscard]] Eigen::Matrix<double, 4, 6> computeRadarJacobian(const Vector6d& state) const;

    /**
     * @brief Computes the Jacobian of the measurement model for EO.
     * @param state The current system state [x, y, z, vx, vy, vz]^T.
     * @return 2x6 Jacobian matrix.
     */
    [[nodiscard]] Eigen::Matrix<double, 2, 6> computeEOJacobian(const Vector6d& state) const;
};