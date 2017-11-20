#ifndef RADAR_H_
#define RADAR_H_
#include "Sensor.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

class Radar : public Sensor {
public:
    Radar();
    ~Radar();
    virtual void enable() override;
    virtual void disable() override;
    virtual void update_measurement(const MeasurementPackage &meas_package) override;
    virtual void update(UKF_fusion& ukf) override;

    virtual void set_std_radr(float val);
    virtual void set_std_radphi(float val);
    virtual void set_std_radrd(float val);
private:
    double std_radr;
    double std_radphi;
    double std_radrd;
};

#endif /* RADAR_H_ */
