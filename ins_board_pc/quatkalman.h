#ifndef QUATKALMAN_H
#define QUATKALMAN_H

#include <QDate>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "wmmwrapper.h"

namespace ublas = boost::numeric::ublas;
using NumMatrix = ublas::matrix<double>;
using NumVector = ublas::vector<double>;
using ZeroMatrix = ublas::zero_matrix<double>;
using ZeroVector = ublas::zero_vector<double>;
using IdentityMatrix = ublas::identity_matrix<double>;

class QuaternionKalman
{
public:
    struct KalmanInput
    {
        NumVector w;
        NumVector a;
        NumVector m;
        QDate day;
        NumVector geo;
        NumVector pos;
        NumVector v;
        double dt;
    };

    struct ProcessNoiseParams
    {
        double gyro_std;
        double gyro_bias_std;
        double accel_std;
    };

    struct MeasurementNoiseParams
    {
        double accel_std;
        double magn_std;
        double gps_cep;
        double gps_vel_abs_std;
    };

    struct FilterParams
    {
        ProcessNoiseParams proc_params;
        MeasurementNoiseParams meas_params;
    };

    QuaternionKalman(const FilterParams & params);

    void step(const KalmanInput & z);

    void reset();

    bool is_initialized();

    NumVector get_state();
    NumVector get_orientation_quaternion();
    NumVector get_gyro_bias();
    NumVector get_position();
    NumVector get_velocity();
    NumVector get_acceleration();
    void get_rpy(double & roll, double & pitch, double & yaw);
    void get_geodetic(double & lat, double & lon, double & alt);

private:    
    void update(const KalmanInput & z);
    void accumulate(const KalmanInput & z);
    void initialize();

    /* create Kalman matrices */
    NumMatrix create_quat_bias_mtx(double dt_2);
    NumMatrix create_transition_mtx(const KalmanInput & z);
    NumMatrix create_proc_noise_cov_mtx(double dt);
    NumMatrix create_meas_noise_cov_mtx(double lat, double lon, double magn_mag);
    NumMatrix create_meas_proj_mtx(double lat, double lon, double alt, QDate day, const NumVector & v);

    /* auxiliary transformations */
    NumMatrix quaternion_to_dcm(const NumVector & quaternion);
    NumMatrix geodetic_to_dcm(double lat, double lon);
    NumVector expected_mag(double lat, double lon, double alt, QDate day);
    double expected_gravity_accel(double lat, double alt);

    NumVector quat_multiply(const NumVector & p, const NumVector & q);

    void normalize_state();
    bool invert_matrix(const NumMatrix & mtx, NumMatrix & inv);

    /* debug functions */
    void debug_vector(const NumVector & vec, QString name);
    void debug_matrix(const NumMatrix & mtx, QString name);

    /* auxiliary derivatives */
    NumMatrix ddcm_dqs(const NumVector & quaternion);
    NumMatrix ddcm_dqx(const NumVector & quaternion);
    NumMatrix ddcm_dqy(const NumVector & quaternion);
    NumMatrix ddcm_dqz(const NumVector & quaternion);

    NumMatrix dcm_lat_partial(double lat, double lon);
    NumMatrix dcm_lon_partial(double lat, double lon);

    /* main derivatives */
    NumMatrix dgeo_dpos(double lat, double lon, double alt);

    /* from state to measurements */
    void calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                                 double lat, double lon, double alt,
                                 double & ax, double & ay, double & az);

    void calculate_magnetometer(const NumVector & orientation_quat,
                                double lat, double lon, double alt, QDate day,
                                double & mx, double & my, double & mz);

    void calculate_geodetic(const NumVector & position,
                            double & lat, double & lon, double & alt);

    void calculate_velocity(const NumVector & velocity, double & vel);

    WrapperWMM wmm;

    NumVector x;
    NumMatrix P;

    bool initialized;

    FilterParams params;

    KalmanInput accum;
    int accum_size;

    static constexpr int accum_capacity = 500;
    static constexpr int state_size = 16;
    static constexpr int measurement_size = 10;
    static constexpr double equator_gravity = 9.7803267714;
    static constexpr double standard_gravity = 9.80665;
    static constexpr double wgs_k = 0.00193185265241;
    static constexpr double wgs_m = 0.00344978650684;
};

#endif // QUATKALMAN_H
