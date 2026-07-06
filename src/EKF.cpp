#include "EKF.h"
#include <cmath>
#include <iostream>
#include <limits>

EKF::EKF() {
    x_.setZero();
    P_.setIdentity();
    
    // Process noise variance (q). 
    // Represents the variance of unknown accelerations (m/s^2)^2.
    // Tuning this parameter is crucial: 
    // Too high -> Filter jitters and follows noise.
    // Too low -> Filter ignores maneuvers and loses the target.
    process_noise_variance_ = 25.0; // Assuming max acceleration of ~5 m/s^2 (5^2 = 25)
}

void EKF::init(const Eigen::VectorXd& z_radar, const Eigen::MatrixXd& R_radar) {
    double r = z_radar(0);
    double az = z_radar(1);
    double el = z_radar(2);
    double vr = z_radar(3);

    // 1. Convert Spherical to Cartesian Position
    double x = r * std::cos(el) * std::cos(az);
    double y = r * std::cos(el) * std::sin(az);
    double z = r * std::sin(el);

    // 2. Initialize Velocity
    // Radar only gives us Radial Velocity (velocity along the line of sight).
    // We project it onto the Cartesian axes as our best initial guess.
    double vx = vr * std::cos(el) * std::cos(az);
    double vy = vr * std::cos(el) * std::sin(az);
    double vz = vr * std::sin(el);

    x_ << x, y, z, vx, vy, vz;

    // 3. Initialize Covariance Matrix (P)
    // We start with the Radar's measurement noise for position, 
    // and a conservatively large uncertainty for velocity (since Vr is incomplete).
    P_.setZero();
    P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 100.0; // 10m std dev
    P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1000.0; // High uncertainty for initial velocity
}

void EKF::predict(double dt) {
    // 1. State Transition Matrix (F) for Constant Velocity (CV) model
    Matrix6d F = Matrix6d::Identity();
    F(0, 3) = dt; // x = x + vx*dt
    F(1, 4) = dt; // y = y + vy*dt
    F(2, 5) = dt; // z = z + vz*dt

    // 2. Process Noise Covariance Matrix (Q)
    // This answers the question: "How does uncertainty grow with time?"
    // Using the Discrete White Noise Acceleration Model.
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    
    // --- ADAPTIVE PROCESS NOISE ---
    // Calculate current velocity magnitude
    double vx = x_(3);
    double vy = x_(4);
    double vz = x_(5);
    double v_mag = std::sqrt(vx*vx + vy*vy + vz*vz);
    
    // The variance 'q' now scales with the speed of the target.
    // Base variance (process_noise_variance_) + adaptive scaling (e.g., adaptive_scaling_factor * speed)
    double adaptive_scaling_factor = 1.0; // Tunable parameter: How much to increase process noise with speed
    double q = process_noise_variance_ + (adaptive_scaling_factor * v_mag);
    
    // ------------------------------

    Matrix6d Q = Matrix6d::Zero();
    
    // Position-Position covariance grows with dt^4
    Q(0, 0) = Q(1, 1) = Q(2, 2) = (dt4 / 4.0) * q;
    
    // Position-Velocity covariance grows with dt^3
    Q(0, 3) = Q(3, 0) = Q(1, 4) = Q(4, 1) = Q(2, 5) = Q(5, 2) = (dt3 / 2.0) * q;
    
    // Velocity-Velocity covariance grows with dt^2
    Q(3, 3) = Q(4, 4) = Q(5, 5) = dt2 * q;

    // 3. Perform Prediction Steps
    // State Prediction: X = F * X
    x_ = F * x_;

    // Covariance Prediction: P = F * P * F^T + Q
    P_ = F * P_ * F.transpose() + Q;
}

// ==========================================
// Helper Function: Angle Normalization
// ==========================================
// Ensures angles are strictly within the [-PI, PI] range.
static void normalizeAngle(double& angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
}

// ==========================================
// Sub-step 2.3: Update & Jacobians
// ==========================================

void EKF::updateRadar(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) {
    // 1. Map predicted Cartesian state to predicted Spherical measurement (h(X))
    double px = x_(0), py = x_(1), pz = x_(2);
    double vx = x_(3), vy = x_(4), vz = x_(5);
    
    double d2 = px*px + py*py;
    double d = std::sqrt(d2);
    double r = std::sqrt(d2 + pz*pz);
    
    // Prevent division by zero if target is exactly at the origin
    if (r < 0.0001) return;

    Eigen::VectorXd z_pred(4);
    z_pred(0) = r;
    z_pred(1) = std::atan2(py, px);
    z_pred(2) = std::asin(pz / r); // Elevation
    z_pred(3) = (px*vx + py*vy + pz*vz) / r; // Radial Velocity

    // 2. Compute Innovation (y = Z - h(X))
    Eigen::VectorXd y = z - z_pred;
    
    // CRITICAL: Normalize angles!
    normalizeAngle(y(1)); // Azimuth difference
    normalizeAngle(y(2)); // Elevation difference

    // 3. Compute Jacobian H
    Eigen::Matrix<double, 4, 6> H = computeRadarJacobian(x_);

    // 4. Standard EKF Math
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + (K * y);
    
    Matrix6d I = Matrix6d::Identity();
    P_ = (I - K * H) * P_;
}

void EKF::updateEO(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) {

    // =========================================================
    // ELEGANT FUSION: Adaptive Measurement Noise (Dynamic R)
    // The further the target, the more we distrust the 2D bearing 
    // due to Line-of-Sight ambiguities. 
    // =========================================================
    double range = std::sqrt(x_(0)*x_(0) + x_(1)*x_(1) + x_(2)*x_(2));
    
    // Scale R based on distance (e.g., at 5000m, R is doubled)
    double scale_factor = 1.0 + (range / 5000.0); 
    Eigen::Matrix2d R_dynamic = R * scale_factor;
    // =========================================================


    double px = x_(0), py = x_(1), pz = x_(2);
    double d2 = px*px + py*py;
    double r = std::sqrt(d2 + pz*pz);

    if (r < 0.0001 || d2 < 0.0001) return;

    // 1. Map state to predicted EO measurement (Angles only)
    Eigen::VectorXd z_pred(2);
    z_pred(0) = std::atan2(py, px);
    z_pred(1) = std::asin(pz / r);

    // 2. Innovation
    Eigen::VectorXd y = z - z_pred;
    normalizeAngle(y(0));
    normalizeAngle(y(1));

    // 3. Compute Jacobian
    Eigen::Matrix<double, 2, 6> H = computeEOJacobian(x_);

    // 4. Update Steps
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_dynamic;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + (K * y);
    
    Matrix6d I = Matrix6d::Identity();
    P_ = (I - K * H) * P_;
}

Eigen::Matrix<double, 4, 6> EKF::computeRadarJacobian(const Vector6d& state) const {
    Eigen::Matrix<double, 4, 6> Hj = Eigen::Matrix<double, 4, 6>::Zero();
    
    double px = state(0), py = state(1), pz = state(2);
    double vx = state(3), vy = state(4), vz = state(5);

    double d2 = px*px + py*py;
    double d = std::sqrt(d2);
    double r2 = d2 + pz*pz;
    double r = std::sqrt(r2);

    if (d2 < 0.0001 || r < 0.0001) {
        return Hj; // Avoid catastrophic divide-by-zero
    }

    // Range derivatives
    Hj(0, 0) = px / r;
    Hj(0, 1) = py / r;
    Hj(0, 2) = pz / r;

    // Azimuth derivatives (d/dx arctan(y/x))
    Hj(1, 0) = -py / d2;
    Hj(1, 1) = px / d2;
    // Hj(1, 2) is 0

    // Elevation derivatives (d/dx arcsin(z/r))
    Hj(2, 0) = -(px * pz) / (r2 * d);
    Hj(2, 1) = -(py * pz) / (r2 * d);
    Hj(2, 2) = d / r2;

    // Radial Velocity derivatives
    double v_dot_p = px*vx + py*vy + pz*vz;
    Hj(3, 0) = vx/r - (px * v_dot_p) / (r2 * r);
    Hj(3, 1) = vy/r - (py * v_dot_p) / (r2 * r);
    Hj(3, 2) = vz/r - (pz * v_dot_p) / (r2 * r);
    Hj(3, 3) = px / r;
    Hj(3, 4) = py / r;
    Hj(3, 5) = pz / r;

    return Hj;
}

Eigen::Matrix<double, 2, 6> EKF::computeEOJacobian(const Vector6d& state) const {
    Eigen::Matrix<double, 2, 6> Hj = Eigen::Matrix<double, 2, 6>::Zero();
    
    double px = state(0), py = state(1), pz = state(2);
    
    double d2 = px*px + py*py;
    double d = std::sqrt(d2);
    double r2 = d2 + pz*pz;

    if (d2 < 0.0001 || r2 < 0.0001) {
        return Hj;
    }

    // Azimuth derivatives (identical to Radar)
    Hj(0, 0) = -py / d2;
    Hj(0, 1) = px / d2;

    // Elevation derivatives (identical to Radar)
    Hj(1, 0) = -(px * pz) / (r2 * d);
    Hj(1, 1) = -(py * pz) / (r2 * d);
    Hj(1, 2) = d / r2;

    return Hj;
}

double EKF::computeMahalanobisRadar(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const {
    double px = x_(0), py = x_(1), pz = x_(2);
    double vx = x_(3), vy = x_(4), vz = x_(5);
    
    double d2 = px*px + py*py;
    double r = std::sqrt(d2 + pz*pz);
    
    // If exact origin, return infinite distance to avoid NaN crash
    if (r < 0.0001) return std::numeric_limits<double>::max();

    Eigen::VectorXd z_pred(4);
    z_pred(0) = r;
    z_pred(1) = std::atan2(py, px);
    z_pred(2) = std::asin(pz / r);
    z_pred(3) = (px*vx + py*vy + pz*vz) / r;

    Eigen::VectorXd y = z - z_pred;
    
    // Use the static helper function defined earlier in the file!
    normalizeAngle(y(1)); 
    normalizeAngle(y(2));

    Eigen::Matrix<double, 4, 6> H = computeRadarJacobian(x_);
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    
    // Calculate Mahalanobis Distance squared: D^2 = y^T * S^-1 * y
    return y.transpose() * S.inverse() * y;
}

double EKF::computeMahalanobisEO(const Eigen::VectorXd& z, const Eigen::MatrixXd& R) const {
    double px = x_(0), py = x_(1), pz = x_(2);
    double d2 = px*px + py*py;
    double r = std::sqrt(d2 + pz*pz);

    if (r < 0.0001 || d2 < 0.0001) return std::numeric_limits<double>::max();

    // =========================================================
    // BUG FIX: Must use the same Dynamic R for Gating!
    // Without this, the gate would use the tiny raw R, causing 
    // valid distant measurements to blow up the Mahalanobis distance
    // and be unfairly rejected by the Chi-Square gate.
    // =========================================================
    double scale_factor = 1.0 + (r / 5000.0); 
    Eigen::Matrix2d R_dynamic = R * scale_factor;

    Eigen::VectorXd z_pred(2);
    z_pred(0) = std::atan2(py, px);
    z_pred(1) = std::asin(pz / r);

    Eigen::VectorXd y = z - z_pred;
    normalizeAngle(y(0));
    normalizeAngle(y(1));

    Eigen::Matrix<double, 2, 6> H = computeEOJacobian(x_);
    
    // CRITICAL: Use R_dynamic here to compute S!
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_dynamic;
    
    return y.transpose() * S.inverse() * y;
}