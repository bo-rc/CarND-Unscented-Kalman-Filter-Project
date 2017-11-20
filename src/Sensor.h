#ifndef SENSOR_H_
#define SENSOR_H_
#include <vector>
#include <string>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "UKF_fusion.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF_fusion;

class Sensor {
public:
    Sensor();
    virtual ~Sensor();

    VectorXd measurement;
    MatrixXd R;
    MeasurementPackage::SensorType type;

    virtual void enable() = 0;
    virtual void disable() = 0;
    virtual void update_measurement(const MeasurementPackage& meas_package) = 0;
    virtual void update(UKF_fusion& ukf) = 0;

protected:
    std::string name;
    bool is_initialized;
    bool is_active;
    bool has_new_data;
    int data_dim;
};

#endif /* SENSOR_H_ */
