#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // boolean flag; to run initialization only once
  is_initialized_ = false;
  time_us_ = 0.0;

  x_.fill(0.0);
  P_.fill(0.0);

  n_aug_  = 7;
  n_x_    = 5;
  lambda_ = 3 - n_aug_;

  // set weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < weights_.size(); i++) {
    weights_(i) =  0.5 / (n_aug_ + lambda_);
  }

  // prediction parameters
  x_aug = VectorXd(n_aug_);

  P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);

  Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  Xsig_aug.fill(0.0);

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  Xsig_pred_.fill(0.0);

  // Laser update -- noise covariance
  R_laser = MatrixXd(2, 2); // 2X2
  R_laser <<  std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  // Radar update -- noise covariance
  R_Radar = MatrixXd(3, 3);
  R_Radar <<    std_radr_*std_radr_, 0, 0,
                0, std_radphi_*std_radphi_, 0,
                0, 0,std_radrd_*std_radrd_;

  // NIS measurements
  NIS_radar = 0.0;
  NIS_laser = 0.0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    x_.fill(0.0);
    P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    time_us_ = meas_package.timestamp_;

    if (meas_package.sensor_type_==MeasurementPackage::LASER && use_laser_) {
      x_[0] = meas_package.raw_measurements_[0];  // px
      x_[1] = meas_package.raw_measurements_[1];  // py
      cout << "\n Initialized with LASER : x_ :\n" << x_ << endl;
    } else if (meas_package.sensor_type_==MeasurementPackage::RADAR && use_radar_) {
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float rho_d = meas_package.raw_measurements_[2];

      float px = rho*cos(phi);
      float py = rho*sin(phi);
      float v  = sqrt(pow(rho_d*cos(phi), 2) + pow(rho_d*sin(phi), 2));

      x_[0] = px;
      x_[1] = py;
      x_[2] = v;
      cout << "\n Initialized with RADAR : x_ :\n" << x_ << endl;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    cout << "\nUKF Init Done!\n";
    return;
  }

  /*  ##################  Control Flow  ##################  */
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; // dt in ms
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  if(meas_package.sensor_type_==MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_==MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
  /*  ##################  ##################  ##################  */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  x_aug.fill(0.0);
  x_aug.head(5) = x_;

  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = pow(std_a_, 2);
  P_aug(6, 6) = pow(std_yawdd_, 2);

  MatrixXd A = P_aug.llt().matrixL();

  Xsig_aug.col(0) = x_aug;
  for (int i=0; i<n_aug_; i++) {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
  }

  MatrixXd temp = MatrixXd(Xsig_aug.cols(), 1);
  temp.fill(0.0);

  for (int i = 0; i<2*n_aug_+1; i++) {
    temp = Xsig_aug.col(i);

    double p_x = temp(0);
    double p_y = temp(1);
    double v = temp(2);
    double yaw = temp(3);
    double yaw_d = temp(4);
    double nu_a = temp(5);
    double nu_y = temp(6);

    double px_p, py_p;

    if (fabs(yaw_d) > 0.0001) {
      px_p = p_x + v / yaw_d * (sin(yaw + delta_t * yaw_d) - sin(yaw));
      py_p = p_y + v / yaw_d * (cos(yaw) - cos(yaw + delta_t * yaw_d));
    } else {
      px_p = p_x + (v * cos(yaw) * delta_t);
      py_p = p_y + (v * sin(yaw) * delta_t);
    }

    double v_p = v;
    double yaw_p = yaw + (yaw_d * delta_t);
    double yaw_d_p = yaw_d;

    // noise
    px_p += 0.5 * pow(delta_t, 2) * cos(yaw) * nu_a;
    py_p += 0.5 * pow(delta_t, 2) * sin(yaw) * nu_a;
    v_p += delta_t * nu_a;
    yaw_p += 0.5 * pow(delta_t, 2) * nu_y;
    yaw_d_p += delta_t * nu_y;

    Xsig_pred_.col(i) << px_p,
            py_p,
            v_p,
            yaw_p,
            yaw_d_p;
  }

  // predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // predict state covariance
  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Normalize(&x_diff(3));

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  VectorXd z = meas_package.raw_measurements_;

  int n_z = 2;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    Zsig(0, i) = Xsig_pred_(0, i);   //p_x
    Zsig(1, i) = Xsig_pred_(1, i);   //p_y
  }

  VectorXd z_pred  = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    z_pred += weights_[i] * Zsig.col(i);
  }

  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_[i] * z_diff * z_diff.transpose();
  }

  S += R_laser;

  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;

  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  NIS_laser = z_diff.transpose() * S.inverse() * z_diff;

//  // Optimize using NIS value
//  cout << "Current NIS for LIDAR -->\t" << NIS_laser << endl;
//  std::ofstream ofs;
//  ofs.open ("lidar_nis.txt", std::ofstream::out | std::ofstream::app);
//  ofs << NIS_laser;
//  ofs << "\n";
//  ofs.close();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  VectorXd z = meas_package.raw_measurements_;

  int n_z = 3;
  MatrixXd temp = MatrixXd(Xsig_pred_.cols(), 1);
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

  for (int i=0; i<2*n_aug_+1; i++) {
    temp = Xsig_pred_.col(i);

    double p_x = temp(0);
    double p_y = temp(1);
    double v = temp(2);
    double yaw = temp(3);

    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = ((p_x*cos(yaw)*v) + (p_y*sin(yaw)*v))/sqrt(p_x*p_x + p_y*p_y);
  }

  VectorXd z_pred  = VectorXd(n_z);
  z_pred.fill(0.0);

  for (int i = 0; i<2*n_aug_+1; i++) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i< 2*n_aug_+1; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;

    Normalize(&z_diff(1));

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  S += R_Radar;

  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i=0; i<2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Normalize(&z_diff(1));
    Normalize(&x_diff(3));

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;

  Normalize(&z_diff(1));

  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  NIS_radar = z_diff.transpose() * S.inverse() * z_diff;

  // Optimize using NIS value
//  cout << "Current NIS for RADAR -->\t" << NIS_radar << endl;
//  std::ofstream ofs;
//  ofs.open ("radar_nis.txt", std::ofstream::out | std::ofstream::app);
//  ofs << NIS_radar;
//  ofs << "\n";
//  ofs.close();
}

void UKF::Normalize(double *angle) {
  //angle normalization; always do so when taking difference between angles.
  while (*angle >  M_PI) *angle-=2.0*M_PI;
  while (*angle < -M_PI) *angle+=2.0*M_PI;
}
