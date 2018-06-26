#include "mixedkalmanfilterbase.h"
#include "earth.h"
#include "geometry.h"
#include "quaternion.h"
#include "packets.h"

MixedKalmanFilterBase::MixedKalmanFilterBase(const Ellipsoid & ellip)
    : KalmanOrientationFilterBase(ellip),
      KalmanPositionFilterBase(ellip),
      PLL{ PLL_type::Zero() },
      PUR{ PUR_type::Zero() }
{}

MixedKalmanFilterBase::meas_type
MixedKalmanFilterBase::do_get_true_measurement(const FilterInput & z) const
{
    meas_type meas;
    meas << KalmanOrientationFilterBase::get_true_measurement(z), KalmanPositionFilterBase::get_true_measurement(z);
    return meas;
}

MixedKalmanFilterBase::meas_type
MixedKalmanFilterBase::do_get_predicted_measurement(const Vector3D & geo, const boost::gregorian::date & day) const
{
    meas_type pred;
    pred << KalmanOrientationFilterBase::get_predicted_measurement(geo, day), KalmanPositionFilterBase::get_predicted_measurement(geo, day);
    return pred;
}

MixedKalmanFilterBase::state_type
MixedKalmanFilterBase::do_get_state() const
{
    state_type state;
    state << KalmanOrientationFilterBase::get_state(), KalmanPositionFilterBase::get_state();
    return state;
}

MixedKalmanFilterBase::P_type
MixedKalmanFilterBase::do_get_cov() const
{
    P_type P;
    P << KalmanOrientationFilterBase::get_cov(), PUR,
         PLL, KalmanPositionFilterBase::get_cov();

    return P;
}

void MixedKalmanFilterBase::do_set_cov(const P_type & cov)
{
    KalmanOrientationFilterBase::set_cov(cov.block<ori_state_size, ori_state_size>(0, 0));
    KalmanPositionFilterBase::set_cov(cov.block<pos_state_size, pos_state_size>(ori_state_size, ori_state_size));
    PUR = cov.block<ori_state_size, pos_state_size>(0, ori_state_size);
    PLL = cov.block<pos_state_size, ori_state_size>(ori_state_size, 0);
}

void MixedKalmanFilterBase::do_set_state(const state_type & st)
{
    KalmanOrientationFilterBase::set_state(st.segment<ori_state_size>(0));
    KalmanPositionFilterBase::set_state(st.segment<pos_state_size>(ori_state_size));
}

Vector3D MixedKalmanFilterBase::do_get_geodetic(const FilterInput & z) const
{
    return KalmanPositionFilterBase::get_geodetic(z);
}

bool MixedKalmanFilterBase::do_is_initialized() const
{
    return KalmanOrientationFilterBase::is_initialized() &&
            KalmanPositionFilterBase::is_initialized();
}

bool MixedKalmanFilterBase::do_is_ready_to_initialize() const
{
    return KalmanOrientationFilterBase::is_ready_to_initialize() &&
            KalmanPositionFilterBase::is_ready_to_initialize();
}

void MixedKalmanFilterBase::do_initialize(const FilterInput & z)
{
    KalmanOrientationFilterBase::initialize(z);
    KalmanPositionFilterBase::initialize(z);
}

void MixedKalmanFilterBase::do_accumulate(const FilterInput & z)
{
    KalmanOrientationFilterBase::accumulate(z);
    KalmanPositionFilterBase::accumulate(z);
}

MixedKalmanFilterBase::F_type
MixedKalmanFilterBase::do_create_transition_mtx(const FilterInput & z) const
{
    F_type F;

    F << KalmanOrientationFilterBase::create_transition_mtx(z), StaticMatrix<ori_state_size, pos_state_size>::Zero(),
            StaticMatrix<pos_state_size, ori_state_size>::Zero(), KalmanPositionFilterBase::create_transition_mtx(z);

    return F;
}

MixedKalmanFilterBase::P_type
MixedKalmanFilterBase::do_create_init_cov_mtx() const
{
    P_type P;

    P << KalmanOrientationFilterBase::create_init_cov_mtx(), StaticMatrix<ori_state_size, pos_state_size>::Zero(),
            StaticMatrix<pos_state_size, ori_state_size>::Zero(), KalmanPositionFilterBase::create_init_cov_mtx();

    return P;
}

MixedKalmanFilterBase::Q_type
MixedKalmanFilterBase::do_create_proc_noise_cov_mtx(double dt) const
{
    Q_type Q;

    Q << KalmanOrientationFilterBase::create_proc_noise_cov_mtx(dt), StaticMatrix<ori_state_size, pos_state_size>::Zero(),
            StaticMatrix<pos_state_size, ori_state_size>::Zero(), KalmanPositionFilterBase::create_proc_noise_cov_mtx(dt);

    return Q;
}

MixedKalmanFilterBase::R_type
MixedKalmanFilterBase::do_create_meas_noise_cov_mtx(const Vector3D & geo, const boost::gregorian::date & day) const
{
    R_type R;

    R << KalmanOrientationFilterBase::create_meas_noise_cov_mtx(geo, day), StaticMatrix<ori_meas_size, pos_meas_size>::Zero(),
            StaticMatrix<pos_meas_size, ori_meas_size>::Zero(), KalmanPositionFilterBase::create_meas_noise_cov_mtx(geo, day);

    return R;
}

MixedKalmanFilterBase::H_type
MixedKalmanFilterBase::do_create_meas_proj_mtx(const Vector3D & geo, const boost::gregorian::date & day) const
{
    using namespace geom;

    // 1
    Matrix3D Dac_Dpos;

    Vector3D a = get_acceleration() / Gravity::gf;
    Matrix3D Clb = get_orientation_quaternion().dcm_tr();

    Matrix3D Dgeo_Dpos = dgeo_dpos(geo, get_ellipsoid());
    Matrix3D Ddcm_Dlat = dcm_lat_partial(geo);
    Matrix3D Ddcm_Dlon = dcm_lon_partial(geo);

    Matrix3D Dcel_Dx = Dgeo_Dpos(0, 0) * Ddcm_Dlat + Dgeo_Dpos(1, 0) * Ddcm_Dlon;
    Matrix3D Dcel_Dy = Dgeo_Dpos(0, 1) * Ddcm_Dlat + Dgeo_Dpos(1, 1) * Ddcm_Dlon;
    Matrix3D Dcel_Dz = Dgeo_Dpos(0, 2) * Ddcm_Dlat + Dgeo_Dpos(1, 2) * Ddcm_Dlon;

    Vector3D col_x = Clb * Dcel_Dx * a;
    Vector3D col_y = Clb * Dcel_Dy * a;
    Vector3D col_z = Clb * Dcel_Dz * a;

    Dac_Dpos << col_x, col_y, col_z;

    // 2
    Matrix3D Cel = geodetic_to_dcm(geo);
    Matrix3D Ceb = Clb * Cel;

    Matrix3D Dac_Da = Ceb / Gravity::gf;

    // Cross filler
    StaticMatrix<ori_meas_size, pos_state_size> UR_filler;
    UR_filler << Dac_Dpos, Matrix3D::Zero(), Dac_Da,
            StaticMatrix<3, pos_state_size>::Zero();

    // Merge
    H_type H;
    H <<    KalmanOrientationFilterBase::create_meas_proj_mtx(geo, day), UR_filler,
            StaticMatrix<pos_meas_size, ori_state_size>::Zero(), KalmanPositionFilterBase::create_meas_proj_mtx(geo, day);

    return H;
}

void MixedKalmanFilterBase::do_reset()
{
    KalmanOrientationFilterBase::do_reset();
    KalmanPositionFilterBase::do_reset();
}
