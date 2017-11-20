#ifndef UKF_FUSION_H
#define UKF_FUSION_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include "Sensor.h"
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

class Sensor;
class UKF_fusion {
public:
    bool use_lidar;
    bool use_radar;
    bool is_initialized;

    ///* state vector: px, py, v, phi, phi_dot
    VectorXd x;

    ///* process covariance matrix
    MatrixXd P;

    ///* Generated sigma points matrix
    MatrixXd Xsig_pts;

    ///* time when the state is true, in us
    long long time_us_;

    ///* delta_t from last time
    float dt;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd;

    ///* Weights of sigma points
    VectorXd weights;

    ///* State dimension
    int n_x;

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
    void ProcessMeasurement(const MeasurementPackage& meas_package, vector<std::unique_ptr<Sensor>>& sensors);

    /**
     * @brief generate augmented sigma points and set sigma_pits
     * @return augmented sigma points
     */
    MatrixXd generate_augmented_sigma_pts();
    MatrixXd predict_sigma_pts(const MatrixXd& Xsig_aug);
    void predict_mean_cov(const MatrixXd& Xsig_pred);
    void predict();
    void update(vector<std::unique_ptr<Sensor>>& sensors);
    void filter(vector<std::unique_ptr<Sensor>>& sensors);
};

#endif /* UKF_FUSION_H */
