#ifndef QUATCOMPLEMENT_H
#define QUATCOMPLEMENT_H

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

    NumVector get_orientation_quaternion() const override;
    NumVector get_gyro_bias() const override;

    void get_rpy(double & roll, double & pitch, double & yaw) const override;

    void set_static_accel_gain(double gain);
    void set_static_magn_gain(double gain);

protected:
    void update(const FilterInput & z) override;
    void initialize(const FilterInput & z);
    void normalize_state() override;

private:
    double calculate_gain(const NumVector & accel) const;

    static const int state_size;

    FilterParams params;
};

#endif
