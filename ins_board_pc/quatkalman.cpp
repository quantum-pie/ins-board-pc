#include "quatkalman.h"

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/assignment.hpp>

#include <QtMath>
#include <QDebug>

#include <QElapsedTimer>

QuaternionKalman::QuaternionKalman(const FilterParams & params)
    : params(params)
{
    reset();
    x = NumVector(state_size);
    P = NumMatrix(state_size, state_size);
}

void QuaternionKalman::accumulate(const KalmanInput & z)
{
    accum.w += z.w;
    accum.a += z.a;
    accum.m += z.m;
    accum.day = z.day;
    accum.geo += z.geo;
    accum.pos += z.pos;
    accum.v += z.v;
    accum.dt = z.dt;

    accum_size++;
}

void QuaternionKalman::initialize()
{
    NumVector gyro_bias = accum.w / accum_size;
    accum.a /= accum_size;
    accum.m /= accum_size;
    accum.pos /= accum_size;
    accum.v /= accum_size;
    accum.geo /= accum_size;

    NumVector a_norm = accum.a / norm_2(accum.a);

    double ax = a_norm[0];
    double ay = a_norm[1];
    double az = a_norm[2];

    NumVector qacc(4);
    if(az >= 0)
    {
        qacc[0] = qSqrt( (az + 1) / 2);
        qacc[1] = - ay / qSqrt(2 * (az + 1));
        qacc[2] = ax / qSqrt(2 * (az + 1));
        qacc[3] = 0;
    }
    else
    {
        qacc[0] = - ay / qSqrt(2 * (1 - az));
        qacc[1] = qSqrt( (1 - az) / 2);
        qacc[2] = 0;
        qacc[3] = ax / qSqrt(2 * (1 - az));
    }

    NumMatrix accel_rotator = qutils::quaternion_to_dcm(qacc);
    NumVector l = prod(accel_rotator, accum.m);

    double lx = l[0];
    double ly = l[1];

    NumVector qmag(4);
    double G = lx * lx + ly * ly;
    double G_sqrt = qSqrt(G);

    if(ly >= 0)
    {
        qmag[0] = qSqrt(G + ly * G_sqrt) / qSqrt(2 * G);
        qmag[1] = 0;
        qmag[2] = 0;
        qmag[3] = - lx / qSqrt(2 * (G + ly * G_sqrt));
    }
    else
    {
        qmag[0] = - lx / qSqrt(2 * (G - ly * G_sqrt));
        qmag[1] = 0;
        qmag[2] = 0;
        qmag[3] = qSqrt(G - ly * G_sqrt) / qSqrt(2 * G);
    }

    NumVector qdecl(4);

    double declination, inclination;
    wmm.measure(accum.geo[0], accum.geo[1], accum.geo[2], accum.day, declination, inclination);

    /* rotate magnetic field by declination degrees around Z axis counterclockwise */
    qdecl[0] = qCos(declination / 2);
    qdecl[1] = 0;
    qdecl[2] = 0;
    qdecl[3] = qSin(declination / 2);

    NumVector tmp = qutils::quat_multiply(qmag, qdecl);
    NumVector q = qutils::quat_multiply(qacc, tmp);

    // from lb to bl quaternion
    x[0] = q[0];
    x[1] = -q[1];
    x[2] = -q[2];
    x[3] = -q[3];

    x[4] = gyro_bias[0];
    x[5] = gyro_bias[1];
    x[6] = gyro_bias[2];

    x[7] = accum.pos[0];
    x[8] = accum.pos[1];
    x[9] = accum.pos[2];

    x[10] = accum.v[0];
    x[11] = accum.v[1];
    x[12] = accum.v[2];

    x[13] = 0;
    x[14] = 0;
    x[15] = 0;

    P = IdentityMatrix(state_size, state_size) * params.init_params.quat_std * params.init_params.quat_std;

    double bias_var = params.init_params.bias_std * params.init_params.bias_std;
    P(4, 4) = bias_var;
    P(5, 5) = bias_var;
    P(6, 6) = bias_var;

    double position_variance = params.init_params.pos_std * params.init_params.pos_std;
    P(7, 7) = position_variance;
    P(8, 8) = position_variance;
    P(9, 9) = position_variance;

    double velocity_variance = params.init_params.vel_std * params.init_params.vel_std;
    P(10, 10) = velocity_variance;
    P(11, 11) = velocity_variance;
    P(12, 12) = velocity_variance;

    double accel_variance = params.init_params.accel_std * params.init_params.accel_std;
    P(13, 13) = accel_variance;
    P(14, 14) = accel_variance;
    P(15, 15) = accel_variance;
}

void QuaternionKalman::reset()
{
    initialized = false;
    accum_size = 0;

    accum.w = ZeroVector(3);
    accum.a = ZeroVector(3);
    accum.m = ZeroVector(3);
    accum.day = QDate();
    accum.pos = ZeroVector(3);
    accum.geo = ZeroVector(3);
    accum.v = ZeroVector(3);
    accum.dt = 0;
}

bool QuaternionKalman::is_initialized()
{
    return initialized;
}

void QuaternionKalman::step(const KalmanInput & z)
{
    if(initialized)
    {
        update(z);
    }
    else
    {
        accumulate(z);
        if(accum_size == accum_capacity)
        {
            initialize();
            initialized = true;
        }
    }
}

void QuaternionKalman::update(const KalmanInput & z)
{
    NumMatrix F = create_transition_mtx(z);
    NumMatrix Q = create_proc_noise_cov_mtx(z.dt);

    //debug_matrix(F, "F");
    //debug_matrix(Q, "Q");
    //debug_matrix(P, "P");

    x = prod(F, x);
    normalize_state();

    NumMatrix tmp = prod(F, P);
    P = prod(tmp, trans(F)) + Q;

    NumVector z_pr(measurement_size);

    double lat, lon, alt;
    NumVector predicted_pos = get_position();
    NumVector predicted_orientation = get_orientation_quaternion();

    calculate_geodetic(predicted_pos, lat, lon, alt);

    double mag_magn = norm_2(z.m);

    calculate_accelerometer(predicted_orientation, get_acceleration(), lat, lon, alt, z_pr(0), z_pr(1), z_pr(2));
    calculate_magnetometer(predicted_orientation, lat, lon, alt, z.day, z_pr(3), z_pr(4), z_pr(5));

    z_pr(6) = predicted_pos(0);
    z_pr(7) = predicted_pos(1);
    z_pr(8) = predicted_pos(2);

    NumVector predicted_v = get_velocity();
    calculate_velocity(predicted_v, z_pr(9));

    NumVector z_meas(measurement_size);
    z_meas <<= z.a, z.m / mag_magn, z.pos, norm_2(z.v);

    NumVector y = z_meas - z_pr;

    //debug_vector(z_meas, "meas");
    //debug_vector(z_pr, "pred");

    NumMatrix R = create_meas_noise_cov_mtx(lat, lon, mag_magn);
    NumMatrix H = create_meas_proj_mtx(lat, lon, alt, z.day, predicted_v);

    //debug_matrix(R, "R");
    //debug_matrix(H, "H");

    tmp = prod(H, P);
    NumMatrix S = prod(tmp, trans(H)) + R;

    NumMatrix S_inv(S.size1(), S.size2());

    if(invert_matrix(S, S_inv))
    {
        tmp = prod(P, trans(H));
        NumMatrix K = prod(tmp, S_inv);

        x += prod(K, y);
        normalize_state();

        tmp = IdentityMatrix(x.size()) - prod(K, H);
        P = prod(tmp, P);
    }

    //debug_vector(get_orientation_quaternion(), "vel");
    //debug_vector(get_gyro_bias(), "gyro bias");
}

void QuaternionKalman::normalize_state()
{
    ublas::project(x, ublas::range(0, 4)) = qutils::quat_normalize(get_orientation_quaternion());
}

NumMatrix QuaternionKalman::create_quat_bias_mtx(double dt_2)
{
    double q_s = x[0];
    double q_x = x[1];
    double q_y = x[2];
    double q_z = x[3];

    NumMatrix K(4, 3);
    K <<= q_x,  q_y,  q_z,
           -q_s,  q_z, -q_y,
           -q_z, -q_s,  q_x,
            q_y, -q_x, -q_s;

    K *= dt_2;

    return K;
}

NumMatrix QuaternionKalman::create_transition_mtx(const KalmanInput & z)
{
    /* useful constants */
    double dt = z.dt;
    double dt_sq = z.dt * z.dt;
    double dt_2 = z.dt / 2;
    double dt_sq_2 = dt_sq / 2;

    /* constructing state transition matrix */
    NumMatrix V = qutils::skew_symmetric(z.w);

    V *= dt_2;
    V += IdentityMatrix(4);

    NumMatrix K = create_quat_bias_mtx(dt_2);

    NumMatrix F(state_size, state_size);

    F <<= V, K, ZeroMatrix(4, 9),
            ZeroMatrix(3, 4), IdentityMatrix(3), ZeroMatrix(3, 9),
            ZeroMatrix(3, 7), IdentityMatrix(3), dt * IdentityMatrix(3), dt_sq_2 * IdentityMatrix(3),
            ZeroMatrix(3, 10), IdentityMatrix(3), dt * IdentityMatrix(3),
            ZeroMatrix(3, 13), IdentityMatrix(3);

    return F;
}

NumMatrix QuaternionKalman::create_proc_noise_cov_mtx(double dt)
{
    /* useful constants */
    double dt_sq = dt * dt;
    double dt_2 = dt / 2;
    double dt_sq_2 = dt_sq / 2;

    NumMatrix K = create_quat_bias_mtx(dt_2);

    NumMatrix Qq = params.proc_params.gyro_std * params.proc_params.gyro_std * prod(K, trans(K));
    NumMatrix Qb = params.proc_params.gyro_bias_std * params.proc_params.gyro_bias_std * IdentityMatrix(3);

    NumMatrix G(9, 3);
    G <<= IdentityMatrix(3) * dt_sq_2,
            IdentityMatrix(3) * dt,
            IdentityMatrix(3);

    NumMatrix Qp = params.proc_params.accel_std * params.proc_params.accel_std * prod(G, trans(G));

    NumMatrix Q(state_size, state_size);

    Q <<= Qq, ZeroMatrix(4, 12),
            ZeroMatrix(3, 4), Qb, ZeroMatrix(3, 9),
            ZeroMatrix(9, 7), Qp;

    return Q;
}

NumMatrix QuaternionKalman::create_meas_noise_cov_mtx(double lat, double lon, double magn_mag)
{
    NumMatrix Ra = params.meas_params.accel_std * params.meas_params.accel_std * IdentityMatrix(3);

    double normalized_magn_std = params.meas_params.magn_std / magn_mag;
    NumMatrix Rm = normalized_magn_std * normalized_magn_std * IdentityMatrix(3);

    double horizontal_linear_std = params.meas_params.gps_cep * 1.2;
    double altitude_std = horizontal_linear_std / 0.53;

    NumMatrix Cel = geodetic_to_dcm(lat, lon);
    NumMatrix local_cov = IdentityMatrix(3);
    local_cov(0, 0) = horizontal_linear_std * horizontal_linear_std;
    local_cov(1, 1) = horizontal_linear_std * horizontal_linear_std;
    local_cov(2, 2) = altitude_std * altitude_std;

    NumMatrix tmp = prod(trans(Cel), local_cov);
    NumMatrix Rp = prod(tmp, Cel);

    NumMatrix R(measurement_size, measurement_size);

    R <<= Ra, ZeroMatrix(3, 7),
            ZeroMatrix(3, 3), Rm, ZeroMatrix(3, 4),
            ZeroMatrix(3, 6), Rp, ZeroMatrix(3, 1),
            ZeroMatrix(1, 9), params.meas_params.gps_vel_abs_std * params.meas_params.gps_vel_abs_std;

    return R;
}

NumMatrix QuaternionKalman::create_meas_proj_mtx(double lat, double lon, double alt, QDate day, const NumVector & v)
{
    // 1
    NumMatrix Dac_Dq(3, 4);

    double height_adjust = expected_gravity_accel(lat, alt) / standard_gravity;

    NumVector a = get_acceleration() / standard_gravity;
    NumVector q = get_orientation_quaternion();

    NumMatrix Cel = geodetic_to_dcm(lat, lon);
    NumMatrix Clb = qutils::quaternion_to_dcm(q);
    NumMatrix Ceb = prod(Clb, Cel);

    NumMatrix Ddcm_Dqs = qutils::ddcm_dqs(q);
    NumMatrix Ddcm_Dqx = qutils::ddcm_dqx(q);
    NumMatrix Ddcm_Dqy = qutils::ddcm_dqy(q);
    NumMatrix Ddcm_Dqz = qutils::ddcm_dqz(q);

    NumVector tmp = prod(Cel, a);

    NumVector col_s = column(Ddcm_Dqs, 2) * height_adjust + prod(Ddcm_Dqs, tmp);
    NumVector col_x = column(Ddcm_Dqx, 2) * height_adjust + prod(Ddcm_Dqx, tmp);
    NumVector col_y = column(Ddcm_Dqy, 2) * height_adjust + prod(Ddcm_Dqy, tmp);
    NumVector col_z = column(Ddcm_Dqz, 2) * height_adjust + prod(Ddcm_Dqz, tmp);

    Dac_Dq <<= ublas::traverse_policy::by_column(), col_s, col_x, col_y, col_z;

    // 2
    NumMatrix Dac_Dpos(3, 3);

    NumMatrix Dgeo_Dpos = dgeo_dpos(lat, lon, alt);
    NumMatrix Ddcm_Dlat = dcm_lat_partial(lat, lon);
    NumMatrix Ddcm_Dlon = dcm_lon_partial(lat, lon);

    NumMatrix Dcel_Dx = Dgeo_Dpos(0, 0) * Ddcm_Dlat + Dgeo_Dpos(1, 0) * Ddcm_Dlon;
    NumMatrix Dcel_Dy = Dgeo_Dpos(0, 1) * Ddcm_Dlat + Dgeo_Dpos(1, 1) * Ddcm_Dlon;
    NumMatrix Dcel_Dz = Dgeo_Dpos(0, 2) * Ddcm_Dlat + Dgeo_Dpos(1, 2) * Ddcm_Dlon;

    tmp = prod(Dcel_Dx, a);
    col_x = prod(Clb, tmp);
    tmp = prod(Dcel_Dy, a);
    col_y = prod(Clb, tmp);
    tmp = prod(Dcel_Dz, a);
    col_z = prod(Clb, tmp);

    Dac_Dpos <<= ublas::traverse_policy::by_column(), col_x, col_y, col_z;

    // 3
    NumMatrix Dac_Da = Ceb / standard_gravity;

    // 4
    NumMatrix Dm_Dq(3, 4);

    NumVector earth_mag = expected_mag(lat, lon, alt, day);

    col_s = prod(Ddcm_Dqs, earth_mag);
    col_x = prod(Ddcm_Dqx, earth_mag);
    col_y = prod(Ddcm_Dqy, earth_mag);
    col_z = prod(Ddcm_Dqz, earth_mag);

    Dm_Dq <<= ublas::traverse_policy::by_column(), col_s, col_x, col_y, col_z;

    // 5
    NumMatrix Dpos_Dpos = IdentityMatrix(3);

    // 6
    NumMatrix Dv_Dv(1, 3);
    double v_abs = norm_2(v);

    if(v_abs > 0)
    {
        Dv_Dv <<= v[0] / v_abs, v[1] / v_abs, v[2] / v_abs;
    }
    else
    {
        Dv_Dv <<= 0, 0, 0;
    }

    // Combine
    NumMatrix H(measurement_size, state_size);
    H <<= Dac_Dq, ZeroMatrix(3, 3), Dac_Dpos, ZeroMatrix(3, 3), Dac_Da,
            Dm_Dq, ZeroMatrix(3, 12),
            ZeroMatrix(3, 7), Dpos_Dpos, ZeroMatrix(3, 6),
            ZeroMatrix(1, 10), Dv_Dv, ZeroMatrix(1, 3);

    return H;
}

void QuaternionKalman::calculate_geodetic(const NumVector & position,
                                          double & lat, double & lon, double & alt)
{
    wmm.cartesian_to_geodetic(position[0], position[1], position[2], lat, lon, alt);
}

void QuaternionKalman::calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                             double lat, double lon, double alt,
                             double & ax, double & ay, double & az)
{
    double height_adjust = expected_gravity_accel(lat, alt) / standard_gravity;

    NumMatrix Clb = qutils::quaternion_to_dcm(orientation_quat);
    NumMatrix Cel = geodetic_to_dcm(lat, lon);
    NumMatrix tmp = prod(Clb, Cel);
    NumVector movement_component = prod(tmp, acceleration / standard_gravity);
    NumVector g(3);
    g <<= 0, 0, height_adjust;

    NumVector gravity_component = prod(Clb, g);

    NumVector rot_accel = gravity_component + movement_component;

    ax = rot_accel(0);
    ay = rot_accel(1);
    az = rot_accel(2);
}

void QuaternionKalman::calculate_magnetometer(const NumVector & orientation_quat,
                                              double lat, double lon, double alt, QDate day,
                                              double & mx, double & my, double & mz)
{
    NumVector rot_magn = prod(qutils::quaternion_to_dcm(orientation_quat),
                                          expected_mag(lat, lon, alt, day));

    mx = rot_magn(0);
    my = rot_magn(1);
    mz = rot_magn(2);
}

void QuaternionKalman::calculate_velocity(const NumVector & velocity, double & vel)
{
    vel = norm_2(velocity);
}

NumMatrix QuaternionKalman::geodetic_to_dcm(double lat, double lon)
{
    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    NumMatrix DCM(3, 3);

    DCM <<= -slon, clon, 0,
            -clon * slat, -slon * slat, clat,
            clon * clat, slon * clat, slat;

    return DCM;
}

NumVector QuaternionKalman::expected_mag(double lat, double lon, double alt, QDate day)
{
    double declination, inclination;
    wmm.measure(lat, lon, alt, day, declination, inclination);

    double sdecl = qSin(declination);
    double cdecl = qCos(declination);
    double sincl = qSin(inclination);
    double cincl = qCos(inclination);

    NumVector RES(3);
    RES <<= sdecl * cincl, cdecl * cincl, -sincl;
    return RES;
}

double QuaternionKalman::expected_gravity_accel(double lat, double alt)
{
    double f = wmm.ellip_f();
    double a = wmm.ellip_a();
    double a_sq = a * a;
    double slat_sq = qPow(qSin(lat), 2);

    double normal_surface_gravity = equator_gravity * (1 + wgs_k * slat_sq) / qSqrt(1 - wmm.ellip_epssq() * slat_sq);
    return normal_surface_gravity * (1 - 2 * alt / a * (1 + f + wgs_m - 2 * f * slat_sq) + 3 * alt * alt / a_sq);
}

bool QuaternionKalman::invert_matrix(const NumMatrix & mtx, NumMatrix & inv)
{
    typedef ublas::permutation_matrix<std::size_t> pmatrix;

    NumMatrix A(mtx);

    // create a permutation matrix for the LU-factorization
    pmatrix pm(A.size1());

    // perform LU-factorization
    int res = lu_factorize(A, pm);
    if (res != 0)
        return false;

    // create identity matrix of "inverse"
    inv.assign(IdentityMatrix(A.size1()));

    // backsubstitute to get the inverse
    lu_substitute(A, pm, inv);

    return true;
}

void QuaternionKalman::debug_vector(const NumVector & vec, QString name)
{
    QDebug deb = qDebug();
    deb << name + ":" << endl;
    for(size_t i = 0; i < vec.size(); ++i)
    {
        deb << vec[i];
    }
    deb << endl;
}

void QuaternionKalman::debug_matrix(const NumMatrix & mtx, QString name)
{
    QDebug deb = qDebug();
    deb << name + ":" << endl;
    for(size_t i = 0; i < mtx.size1(); ++i)
    {
        for(size_t j = 0; j < mtx.size2(); ++j)
        {
            deb << mtx(i, j);
        }
        deb << endl;
    }
}

NumMatrix QuaternionKalman::dcm_lat_partial(double lat, double lon)
{
    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    NumMatrix RES(3, 3);

    RES <<= 0, 0, 0,
            -clon * clat, -slon * clat, -slat,
            -clon * slat, -slon * slat, clat;

    return RES;
}

NumMatrix QuaternionKalman::dcm_lon_partial(double lat, double lon)
{
    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    NumMatrix RES(3, 3);

    RES <<= -clon, -slon, 0,
            slon * slat, -clon * slat, 0,
            -slon * clat, clon * clat, 0;

    return RES;
}

NumMatrix QuaternionKalman::dgeo_dpos(double lat, double lon, double alt)
{
    double clat = qCos(lat);
    double slat = qSin(lat);
    double clon = qCos(lon);
    double slon = qSin(lon);

    double bracket = qPow(1 - wmm.ellip_epssq() * slat * slat, 1.5);
    double norm_rad = wmm.ellip_a() / qSqrt(1 - wmm.ellip_epssq() * slat * slat);
    double common_mult = bracket / (wmm.ellip_a() * (wmm.ellip_epssq() - 1) - alt * bracket);
    double common_mult_2 = 1 / (norm_rad + alt);

    NumMatrix RES(3, 3);

    RES(0, 0) = common_mult / (slat * clon);
    RES(0, 1) = common_mult / (slat * slon);
    RES(0, 2) = -common_mult / clat;

    RES(1, 0) = -common_mult_2 / (clat * slon);
    RES(1, 1) = common_mult_2 / (clat * clon);
    RES(1, 2) = 0;

    RES(2, 0) = 1 / (clat * clon);
    RES(2, 1) = 1 / (clat * slon);
    RES(2, 2) = 1 / slat;

    return RES;
}

NumVector QuaternionKalman::get_state()
{
    return x;
}

NumVector QuaternionKalman::get_orientation_quaternion()
{
    return ublas::project(x, ublas::range(0, 4));
}

NumVector QuaternionKalman::get_gyro_bias()
{
    return ublas::project(x, ublas::range(4, 7));
}

NumVector QuaternionKalman::get_position()
{
    return ublas::project(x, ublas::range(7, 10));
}

NumVector QuaternionKalman::get_velocity()
{
    return ublas::project(x, ublas::range(10, 13));
}

NumVector QuaternionKalman::get_acceleration()
{
    return ublas::project(x, ublas::range(13, 16));
}

void QuaternionKalman::get_rpy(double & roll, double & pitch, double & yaw)
{
    qutils::quat_to_rpy(get_orientation_quaternion(), roll, pitch, yaw);
}

void QuaternionKalman::get_geodetic(double & lat, double & lon, double & alt)
{
    calculate_geodetic(get_position(), lat, lon, alt);
}

void QuaternionKalman::set_proc_gyro_std(double std)
{
    params.proc_params.gyro_std = std;
}

void QuaternionKalman::set_proc_gyro_bias_std(double std)
{
    params.proc_params.gyro_bias_std = std;
}

void QuaternionKalman::set_proc_accel_std(double std)
{
    params.proc_params.accel_std = std;
}

void QuaternionKalman::set_meas_accel_std(double std)
{
    params.meas_params.accel_std = std;
}

void QuaternionKalman::set_meas_magn_std(double std)
{
    params.meas_params.magn_std = std;
}

void QuaternionKalman::set_meas_pos_std(double std)
{
    params.meas_params.gps_cep = std;
}

void QuaternionKalman::set_meas_vel_std(double std)
{
    params.meas_params.gps_vel_abs_std = std;
}

void QuaternionKalman::set_init_quat_std(double std)
{
    params.init_params.quat_std = std;
}

void QuaternionKalman::set_init_bias_std(double std)
{
    params.init_params.bias_std = std;
}

void QuaternionKalman::set_init_pos_std(double std)
{
    params.init_params.pos_std = std;
}

void QuaternionKalman::set_init_vel_std(double std)
{
    params.init_params.vel_std = std;
}

void QuaternionKalman::set_init_accel_std(double std)
{
    params.init_params.accel_std = std;
}
