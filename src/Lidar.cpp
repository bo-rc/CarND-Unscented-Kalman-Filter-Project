#include "Lidar.h"


Lidar::Lidar()
{
    name = "lidar";
    type = MeasurementPackage::SensorType::LASER;
    data_dim = 2;
    std_px = 0.15;
    std_py = 0.15;
    measurement = Vector2d();
    R = MatrixXd(2,2);
    R << std_px*std_px, 0,
	 0, std_py*std_py;
    enable();
}

Lidar::~Lidar()
{
    // nothing to do
}

void Lidar::enable()
{
    is_active = true;
}

void Lidar::disable()
{
    is_active = false;
}

void Lidar::set_std_px(float val)
{
    std_px = val;
}

void Lidar::set_std_py(float val)
{
    std_py = val;
}

void Lidar::update_measurement(const MeasurementPackage & meas_package)
{
    if ((meas_package.sensor_type_ != type) || !is_active) return;

    auto px = meas_package.raw_measurements_[0];
    auto py = meas_package.raw_measurements_[1];

    measurement << px, py;
    if (!is_initialized) is_initialized = true;
    has_new_data = true;
}

void Lidar::update(UKF_fusion& ukf)
{
    // no new data, skip this sensor
    if (!has_new_data || !is_active) return;

    int n_x = ukf.n_x;
    int n_aug = ukf.n_x + 2;
    int n_z = 2;
    VectorXd weights = ukf.weights;
    MatrixXd Xsig_pred = ukf.Xsig_pts;
    VectorXd x = ukf.x;
    MatrixXd P = ukf.P;

    /* determine sigma points in the measurement space */
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug + 1; i++) {
      // extract values for better readibility
      double p_x = Xsig_pred(0,i);
      double p_y = Xsig_pred(1,i);

      // measurement model
      Zsig(0,i) = p_x;
      Zsig(1,i) = p_y;
    }

    /* recover mean and cov for predicted measurement */
    // mean
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug+1; i++) {
	z_pred = z_pred + weights(i) * Zsig.col(i);
    }

    // covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {
      //residual
      VectorXd z_diff = Zsig.col(i) - z_pred;

      S = S + weights(i) * z_diff * z_diff.transpose();
    }

    // additive measurement noise
    S = S + R;

    /* update state: x and P */
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x, n_z);

    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {

      //residual
      VectorXd z_diff = Zsig.col(i) - z_pred;
      // state difference
      VectorXd x_diff = Xsig_pred.col(i) - x;

      Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = measurement - z_pred;

    //update state mean and covariance matrix
    x = x + K * z_diff;
    P = P - K*S*K.transpose();

    ukf.x = x;
    ukf.P = P;

    has_new_data = false;
}
