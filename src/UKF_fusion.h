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
    ///* state vector: px, py, v, phi, phi_dot
    VectorXd x;

    ///* process covariance matrix
    MatrixXd P;

    ///* Generated sigma points matrix
    MatrixXd Xsig_pts;

    ///* Weights of sigma points
    VectorXd weights;

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

    void enable_lidar();
    void enable_radar();
    void disable_lidar();
    void disable_radar();
    bool is_use_lidar();
    bool is_use_radar();
    int get_x_dim();

    void set_std_a(float val);
    void set_std_yawdd(float val);
private:
    bool use_lidar;
    bool use_radar;
    bool is_initialized;
    long long time_us_; // system time in microseconds
    float dt;
    ///* State dimension
    int n_x;
    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a;
    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd;
    ///* Sigma point spreading parameter
    double lambda;
};

#endif /* UKF_FUSION_H */
