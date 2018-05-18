#ifndef MIXEDKALMANFILTERINGCONTROLLER_H
#define MIXEDKALMANFILTERINGCONTROLLER_H

#include "filtering/filters/generickalman.h"

#include <QObject>

class KalmanOrientationFilteringController;
class KalmanPositionFilteringController;

class KalmanFilteringMetaController : public QObject
{
    Q_OBJECT

public:
    KalmanFilteringMetaController(KalmanOrientationFilteringController & ori_ctrl,
                                  KalmanPositionFilteringController & pos_ctrl);

public slots:
    void handle_start(bool en);
    void handle_type(int combobox_idx);
    void handle_mix_strategy(int combobox_idx);
    void handle_input(const FilterInput & z);

private:
    bool is_mixed_type;

    KalmanOrientationFilteringController & ori_ctrl;
    KalmanPositionFilteringController & pos_ctrl;

    FullEKF ekf_filter;
    FullUKF ukf_filter;
};

#endif // MIXEDKALMANFILTERINGCONTROLLER_H
