#ifndef LIDAR_H_
#define LIDAR_H_
#include "Sensor.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;

class Lidar : public Sensor {
public:
    Lidar();
    ~Lidar();
    virtual void enable() override;
    virtual void disable() override;
    virtual void update_measurement(const MeasurementPackage &meas_package) override;
    virtual void update(UKF_fusion& ukf) override;

    void set_std_px(float val);
    void set_std_py(float val);

private:
    double std_px;
    double std_py;
};

#endif /* LIDAR_H_ */
