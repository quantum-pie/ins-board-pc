#ifndef QUATKALMAN_H
#define QUATKALMAN_H

#include <QDate>

#include "ublasaux.h"
#include "quaternions.h"
#include "qualitycontrol.h"

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
    void initialize(const KalmanInput & z);
    bool bias_estimated();

    void normalize_state();

    /* create Kalman matrices */
    NumMatrix create_transition_mtx(const KalmanInput & z);
    NumMatrix create_proc_noise_cov_mtx(double dt);
    NumMatrix create_meas_noise_cov_mtx(double lat, double lon, double magn_mag);
    NumMatrix create_meas_proj_mtx(double lat, double lon, double alt, QDate day, const NumVector & v);

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

    NumVector x;
    NumMatrix P;

    bool initialized;

    FilterParams params;

    QualityControl bias_x_ctrl;
    QualityControl bias_y_ctrl;
    QualityControl bias_z_ctrl;

    const int accum_capacity = 500;
    const int state_size = 16;
    const int measurement_size = 10;
};

#endif // QUATKALMAN_H
