#ifndef QUATKALMAN_H
#define QUATKALMAN_H

#include <QDate>

#include "ublastypes.h"
#include "quaternions.h"

#include "wmmwrapper.h"

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

    struct InitCovParams
    {
        double quat_std;
        double bias_std;
        double pos_std;
        double vel_std;
        double accel_std;
    };

    struct FilterParams
    {
        ProcessNoiseParams proc_params;
        MeasurementNoiseParams meas_params;
        InitCovParams init_params;
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

    void set_proc_gyro_std(double std);
    void set_proc_gyro_bias_std(double std);
    void set_proc_accel_std(double std);

    void set_meas_accel_std(double std);
    void set_meas_magn_std(double std);
    void set_meas_pos_std(double std);
    void set_meas_vel_std(double std);

    void set_init_quat_std(double std);
    void set_init_bias_std(double std);
    void set_init_pos_std(double std);
    void set_init_vel_std(double std);
    void set_init_accel_std(double std);

private:    
    void update(const KalmanInput & z);
    void accumulate(const KalmanInput & z);
    void initialize();
    void normalize_state();

    /* create Kalman matrices */
    NumMatrix create_quat_bias_mtx(double dt_2);
    NumMatrix create_transition_mtx(const KalmanInput & z);
    NumMatrix create_proc_noise_cov_mtx(double dt);
    NumMatrix create_meas_noise_cov_mtx(double lat, double lon, double magn_mag);
    NumMatrix create_meas_proj_mtx(double lat, double lon, double alt, QDate day, const NumVector & v);

    /* auxiliary transformations */
    NumMatrix geodetic_to_dcm(double lat, double lon);
    NumVector expected_mag(double lat, double lon, double alt, QDate day);
    double expected_gravity_accel(double lat, double alt);

    bool invert_matrix(const NumMatrix & mtx, NumMatrix & inv);

    /* debug functions */
    void debug_vector(const NumVector & vec, QString name);
    void debug_matrix(const NumMatrix & mtx, QString name);

    /* auxiliary derivatives */
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

    const int accum_capacity = 500;
    const int state_size = 16;
    const int measurement_size = 10;
    const double equator_gravity = 9.7803267714;
    const double standard_gravity = 9.80665;
    const double wgs_k = 0.00193185265241;
    const double wgs_m = 0.00344978650684;
};

#endif // QUATKALMAN_H
