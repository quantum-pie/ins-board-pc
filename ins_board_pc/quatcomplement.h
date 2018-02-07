#ifndef QUATCOMPLEMENT_H
#define QUATCOMPLEMENT_H

#include <QString>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "ublasaux.h"
#include "quaternions.h"
#include "qualitycontrol.h"

class QuatComplement
{
public:
    struct ComplementInput
    {
        NumVector w;
        NumVector a;
        NumVector m;
        double dt;
    };

    struct FilterParams
    {
        double static_accel_gain;
        double static_magn_gain;
    };

    QuatComplement(const FilterParams & params);

    void step(const ComplementInput & z);

    void reset();

    bool is_initialized();

    NumVector get_state();
    NumVector get_orientation_quaternion();
    NumVector get_gyro_bias();

    void get_rpy(double & roll, double & pitch, double & yaw);

    void set_static_accel_gain(double gain);
    void set_static_magn_gain(double gain);

private:
    void update(const ComplementInput & z);
    void accumulate(const ComplementInput & z);
    void initialize(const ComplementInput & z);
    bool bias_estimated();

    void normalize_state();
    double calculate_gain(const NumVector & accel);

    static const int accum_capacity = 500;
    static const int state_size = 7;

    NumVector x;
    bool initialized;

    FilterParams params;

    QualityControl bias_x_ctrl;
    QualityControl bias_y_ctrl;
    QualityControl bias_z_ctrl;
};

#endif
