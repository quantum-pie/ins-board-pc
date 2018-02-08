#ifndef QUATCOMPLEMENT_H
#define QUATCOMPLEMENT_H

#include <QString>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "abstractorientationfilter.h"

class QuaternionComplement final : public AbstractOrientationFilter
{
public:
    struct FilterParams
    {
        double static_accel_gain;
        double static_magn_gain;
        int accum_capacity;
    };

    QuaternionComplement(const FilterParams & params);
    ~QuaternionComplement() override;

    void step(const FilterInput & z) override;

    NumVector get_orientation_quaternion() override;
    NumVector get_gyro_bias() override;

    void get_rpy(double & roll, double & pitch, double & yaw) override;

    void set_static_accel_gain(double gain);
    void set_static_magn_gain(double gain);

protected:
    void update(const FilterInput & z) override;
    void initialize(const FilterInput & z);
    void normalize_state() override;

private:
    double calculate_gain(const NumVector & accel);

    static const int state_size = 7;

    FilterParams params;
};

#endif
