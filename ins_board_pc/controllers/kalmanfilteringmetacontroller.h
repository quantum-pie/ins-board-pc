#ifndef MIXEDKALMANFILTERINGCONTROLLER_H
#define MIXEDKALMANFILTERINGCONTROLLER_H

#include "filtering/filters/generickalman.h"
#include "controllers/kalmanorientationfilteringcontroller.h"
#include "controllers/kalmanpositionfilteringcontroller.h"

#include <QObject>

class KalmanFilteringMetaController : public QObject
{
    Q_OBJECT

public:
    KalmanFilteringMetaController(KalmanOrientationFilteringController & ori_ctrl,
                                  KalmanPositionFilteringController & pos_ctrl)
        : is_mixed_type{ false }, ori_ctrl{ ori_ctrl }, pos_ctrl{ pos_ctrl } {}

public slots:
    void handle_start(bool en)
    {
        ori_ctrl.handle_start(en);
        pos_ctrl.handle_start(en);
    }

    void handle_type(int combobox_idx)
    {
        is_mixed_type = (combobox_idx == 0);
    }

    void handle_strategy(int combobox_idx)
    {
        if(combobox_idx == 0)
        {
            ori_ctrl.handle_strategy(&ekf_filter);
            pos_ctrl.handle_strategy(&ekf_filter);
        }
        else
        {
            ori_ctrl.handle_strategy(&ukf_filter);
            pos_ctrl.handle_strategy(&ukf_filter);
        }
    }

    void handle_input(const FilterInput & z)
    {
        ori_ctrl.handle_input(z);
        if(!is_mixed_type)
        {
            pos_ctrl.handle_input(z);
        }
    }

private:
    bool is_mixed_type;

    KalmanOrientationFilteringController & ori_ctrl;
    KalmanPositionFilteringController & pos_ctrl;

    FullEKF ekf_filter;
    FullUKF ukf_filter;
};

#endif // MIXEDKALMANFILTERINGCONTROLLER_H
