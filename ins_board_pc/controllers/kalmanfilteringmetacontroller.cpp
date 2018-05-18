#include "controllers/kalmanfilteringmetacontroller.h"
#include "controllers/kalmanorientationfilteringcontroller.h"
#include "controllers/kalmanpositionfilteringcontroller.h"

KalmanFilteringMetaController::KalmanFilteringMetaController(KalmanOrientationFilteringController &ori_ctrl,
                                                             KalmanPositionFilteringController &pos_ctrl)
    : is_mixed_type{ false }, ori_ctrl{ ori_ctrl }, pos_ctrl{ pos_ctrl }
{}

void KalmanFilteringMetaController::handle_start(bool en)
{
    ori_ctrl.handle_start(en);
    pos_ctrl.handle_start(en);
}

void KalmanFilteringMetaController::handle_type(int combobox_idx)
{
    is_mixed_type = (combobox_idx == 0);
}

void KalmanFilteringMetaController::handle_strategy(int combobox_idx)
{
    if(combobox_idx == 0)
    {
        ori_ctrl.handle_ko_strategy(&ekf_filter);
        pos_ctrl.handle_kp_strategy(&ekf_filter);
    }
    else
    {
        ori_ctrl.handle_ko_strategy(&ukf_filter);
        pos_ctrl.handle_kp_strategy(&ukf_filter);
    }
}

void KalmanFilteringMetaController::handle_input(const FilterInput & z)
{
    ori_ctrl.handle_input(z);
    if(!is_mixed_type)
    {
        pos_ctrl.handle_input(z);
    }
}
