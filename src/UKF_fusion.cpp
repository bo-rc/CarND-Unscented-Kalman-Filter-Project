#include "UKF_fusion.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF_fusion::UKF_fusion() :
    use_lidar(true),
    use_radar(true),
    is_initialized(false),
    time_us_(0),
    dt(0),
    n_x(5),
    std_a(0.75),
    std_yawdd(0.5)
{
   /* the initialization. */
    x = VectorXd(5);
    x << 0.6,0.6,5.0,0,0;

    P = MatrixXd(5,5);
    P <<   0.00354799,  0.00154688,  0.00517556, 0.000588704,  0.00036624,
	   0.00154688,  0.00236147,   0.0032661, 0.000584431, 0.000235127,
	   0.00517556,   0.0032661,    0.016177,  0.00195641,  0.0023692,
	   0.000588704, 0.000584431,  0.00195641, 0.000518974, 0.000992952,
	   0.00036624, 0.000235127,   0.0023692, 0.000992952,   0.0047128;

    auto n_aug = n_x + 2; // 2D noise vector
    Xsig_pts = MatrixXd(n_x, 2 * n_aug + 1);
    lambda = 3- n_aug;

    weights = VectorXd(2*n_aug + 1);
    // set weights
    double weight_0 = lambda/(lambda+n_aug);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug+1; i++) {
      double weight = 0.5/(n_aug+lambda);
      weights(i) = weight;
    }
}

UKF_fusion::~UKF_fusion() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF_fusion::ProcessMeasurement(const MeasurementPackage& meas_package, vector<std::unique_ptr<Sensor>>& sensors) {

    if (!is_initialized) {
	time_us_ = meas_package.timestamp_;
	is_initialized = true;
	return;
    }

    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER && !use_lidar) return;
    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && !use_radar) return;

    dt = (meas_package.timestamp_ - time_us_) / 1000000.0; // dt in seconds
    time_us_ = meas_package.timestamp_;

    for (auto& sensor : sensors) {
	sensor->update_measurement(meas_package);
    }
}


MatrixXd UKF_fusion::generate_augmented_sigma_pts()
{
    auto n_aug = n_x + 2;  // 2D process noise

    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug, n_aug);

    MatrixXd Xsig_aug(n_aug, 2*n_aug + 1);

    //create augmented mean state
    x_aug.head(n_x) = x;
    x_aug(n_x + 0) = 0;
    x_aug(n_x + 1) = 0;

    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x,n_x) = P;
    P_aug(n_x + 0,n_x + 0) = std_a*std_a;
    P_aug(n_x + 1,n_x + 1) = std_yawdd*std_yawdd;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug; i++)
    {
      Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
      Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
    }

    return Xsig_aug;
}

MatrixXd UKF_fusion::predict_sigma_pts(const MatrixXd& Xsig_aug)
{
    auto n_aug = n_x + 2; // 2D noise

    //create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

    //predict sigma points
    for (int i = 0; i< 2*n_aug+1; i++)
    {
      //extract values for better readability
      double p_x = Xsig_aug(0,i);
      double p_y = Xsig_aug(1,i);
      double v = Xsig_aug(2,i);
      double yaw = Xsig_aug(3,i);
      double yawd = Xsig_aug(4,i);
      double nu_a = Xsig_aug(5,i);
      double nu_yawdd = Xsig_aug(6,i);

      //predicted state values
      double px_p, py_p;

      //avoid division by zero
      if (fabs(yawd) > 0.001) {
	  px_p = p_x + v/yawd * ( sin (yaw + yawd*dt) - sin(yaw));
	  py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*dt) );
      }
      else {
	  px_p = p_x + v*dt*cos(yaw);
	  py_p = p_y + v*dt*sin(yaw);
      }

      double v_p = v;
      double yaw_p = yaw + yawd*dt;
      double yawd_p = yawd;

      //add noise
      px_p = px_p + 0.5*nu_a*dt*dt * cos(yaw);
      py_p = py_p + 0.5*nu_a*dt*dt * sin(yaw);
      v_p = v_p + nu_a*dt;

      yaw_p = yaw_p + 0.5*nu_yawdd*dt*dt;
      yawd_p = yawd_p + nu_yawdd*dt;

      //write predicted sigma point into right column
      Xsig_pred(0,i) = px_p;
      Xsig_pred(1,i) = py_p;
      Xsig_pred(2,i) = v_p;
      Xsig_pred(3,i) = yaw_p;
      Xsig_pred(4,i) = yawd_p;
    }

    // update predicted sigma points
    Xsig_pts = Xsig_pred;

    return Xsig_pred;
}

void UKF_fusion::predict_mean_cov(const MatrixXd& Xsig_pred) {

  auto n_aug = n_x + 2; // 2D noise

  //create vector for predicted state
  VectorXd x_out = VectorXd(n_x);

  //create covariance matrix for prediction
  MatrixXd P_out = MatrixXd(n_x, n_x);

  //predicted state mean
  x_out.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
    x_out = x_out + weights(i) * Xsig_pred.col(i);
  }

  //predicted state covariance matrix
  P_out.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_out;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_out = P_out + weights(i) * x_diff * x_diff.transpose() ;
  }

  // update augument
  x = x_out;
  P = P_out;
}

void UKF_fusion::predict()
{
    MatrixXd Xsig_aug = generate_augmented_sigma_pts();
    MatrixXd Xsig_pred = predict_sigma_pts(Xsig_aug);
    predict_mean_cov(Xsig_pred);
}

void UKF_fusion::update(vector<std::unique_ptr<Sensor>>& sensors)
{
    for (auto &sensor: sensors) {
	sensor->update(*this);
    }
}

void UKF_fusion::filter(vector<std::unique_ptr<Sensor>>& sensors)
{
    predict();
    update(sensors);
}

void UKF_fusion::enable_lidar()
{
    use_lidar = true;
}

void UKF_fusion::enable_radar()
{
    use_radar = true;
}

void UKF_fusion::disable_lidar()
{
    use_lidar = false;
}

void UKF_fusion::disable_radar()
{
    use_radar = false;
}

bool UKF_fusion::is_use_lidar()
{
    return use_lidar;
}

bool UKF_fusion::is_use_radar()
{
    return use_radar;
}

void UKF_fusion::set_std_a(float val)
{
    std_a = val;
}

void UKF_fusion::set_std_yawdd(float val)
{
    std_yawdd = val;
}

int UKF_fusion::get_x_dim()
{
    return n_x;
}
