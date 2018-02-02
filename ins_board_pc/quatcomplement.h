#ifndef QUATCOMPLEMENT_H
#define QUATCOMPLEMENT_H

#include <QString>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace ublas = boost::numeric::ublas;
using NumMatrix = ublas::matrix<double>;
using NumVector = ublas::vector<double>;
using ZeroMatrix = ublas::zero_matrix<double>;
using ZeroVector = ublas::zero_vector<double>;
using IdentityMatrix = ublas::identity_matrix<double>;

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
    void initialize();

    NumMatrix create_quat_bias_mtx(double dt_2);

    /* auxiliary transformations */
    NumMatrix quaternion_to_dcm(const NumVector & quaternion);

    NumVector quat_multiply(const NumVector & p, const NumVector & q);

    void normalize_state();

    /* debug functions */
    void debug_vector(const NumVector & vec, QString name);
    void debug_matrix(const NumMatrix & mtx, QString name);

    NumVector x;
    bool initialized;

    FilterParams params;

    ComplementInput accum;
    int accum_size;

    const int accum_capacity = 500;
    const int state_size = 7;
    const double standard_gravity = 9.80665;
};

#endif
