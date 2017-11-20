#ifndef UKF_FUSION_H
#define UKF_FUSION_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include "Sensor.h"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

class UKF_fusion {
public:
    bool use_lidar;
    bool use_radar;

    ///* state vector: px, py, v, phi, phi_dot
    VectorXd x;

    ///* state covariance matrix
    MatrixXd P;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred;

    ///* time when the state is true, in us
    long long time_us_;

    ///* delta_t from last time
    float dt;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawd;

    ///* lidar measurement noise standard deviation position1 in m
    double std_lidpx;

    ///* lidar measurement noise standard deviation position2 in m
    double std_lidpy;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd;

    ///* Weights of sigma points
    VectorXd weights;

    ///* State dimension
    int n_x;

    ///* Augmented state dimension
    int n_aug;

    ///* Sigma point spreading parameter
    double lambda;

    /**
     * Constructor
     */
    UKF_fusion();

    /**
     * Destructor
     */
    virtual ~UKF_fusion();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or lidar
     */
    void ProcessMeasurement(MeasurementPackage meas_package);

  private:
    const double PI = 3.14159265;
    vector<Sensor> sensors;
};

#endif /* UKF_FUSION_H */
