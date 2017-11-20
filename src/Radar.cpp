#include "Radar.h"
#include <iostream>
using std::cout;
using std::endl;


Radar::Radar():
    Sensor()
{
    name = "radar0";
    data_dim = 3;
    std_radr = 0.3;
    std_radphi = 0.03;
    std_radrd = 0.3;

    type = MeasurementPackage::SensorType::RADAR;

    measurement = Vector3d();

    R = MatrixXd(3,3);
    R << std_radr*std_radr, 0, 0,
	 0, std_radphi*std_radphi, 0,
	 0, 0, std_radrd*std_radrd;

    enable();
}

Radar::~Radar()
{
    // nothing to do
}

void Radar::enable()
{
    is_active = true;
}

void Radar::disable()
{
    is_active = false;
}

void Radar::set_std_radr(float val)
{
    std_radr = val;
}

void Radar::set_std_radphi(float val)
{
    std_radphi = val;
}

void Radar::set_std_radrd(float val)
{
    std_radrd = val;
}

void Radar::update_measurement(const MeasurementPackage &meas_package)
{
    if ((meas_package.sensor_type_ != type) || !is_active) return;

    auto rho = meas_package.raw_measurements_[0];
    auto phi = meas_package.raw_measurements_[1];
    auto rho_dot = meas_package.raw_measurements_[2];

    while(phi > M_PI) phi -= 2.*M_PI;
    while(phi < -M_PI) phi += 2.*M_PI;

    measurement << rho, phi, rho_dot;

    if (!is_initialized) is_initialized = true;

    has_new_data = true;
}

void Radar::update(UKF_fusion& ukf)
{
    // no new data, skip this sensor
    if (!has_new_data || !is_active) return;

    int n_x = ukf.get_x_dim();
    int n_aug = n_x + 2;
    int n_z = 3;
    VectorXd weights = ukf.weights;
    MatrixXd Xsig_pred = ukf.Xsig_pts;
    VectorXd x = ukf.x;
    MatrixXd P = ukf.P;

    /* determine sigma points in the measurement space */
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug + 1; i++) {
      // extract values for better readibility
      double px = Xsig_pred(0,i);
      double py = Xsig_pred(1,i);
      double v  = Xsig_pred(2,i);
      double yaw = Xsig_pred(3,i);
      double vx = cos(yaw)*v;
      double vy = sin(yaw)*v;

      // measurement model
      double rho = sqrt(px*px + py*py);

      double phi = 0;
      if (fabs(rho) > 0.00001) {
	  phi = asin(py/rho);

	  if (px >= 0) {
	      if (py >= 0) {
		  phi = phi;
	      } else {
		  phi = phi;
	      }
	  } else {
	      if (py >= 0) {
		  phi = M_PI - phi;
	      } else {
		  phi = - (M_PI + phi);
	      }
	  }
      }

      double rho_dot = 0;
      if (fabs(rho) > 0.00001) {
	  rho_dot = (px*vx + py*vy) / rho;
      }

      Zsig(0,i) = rho;
      Zsig(1,i) = phi;
      Zsig(2,i) = rho_dot;
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

      //angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

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
      //angle normalization
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

      // state difference
      VectorXd x_diff = Xsig_pred.col(i) - x;
      //angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

      Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = measurement - z_pred;

    // cout << endl;
    // cout << "measurement = " << measurement[0] << "," << measurement[1] << "," << measurement[2] << endl;
    // cout << "     z_pred = " << z_pred[0] << "," << z_pred[1] << "," << z_pred[2] << endl;
    // cout << endl;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x = x + K * z_diff;
    P = P - K*S*K.transpose();

    ukf.x = x;
    ukf.P = P;
    // cout << P << endl;

    has_new_data = false;
}
