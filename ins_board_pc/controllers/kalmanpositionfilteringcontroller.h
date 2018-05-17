#ifndef KALMANPOSITIONFILTERINGCONTROLLER_H
#define KALMANPOSITIONFILTERINGCONTROLLER_H

#include "filtering/filters/generickalman.h"

#include <QObject>


class FilterInput;

class KalmanPositionFilteringController : public QObject
{
    Q_OBJECT

public:
    KalmanPositionFilteringController(QObject * parent = nullptr);

public slots:
    void handle_start(bool en);
    void handle_strategy(int idx);
    void handle_input(const FilterInput & z);

private:
    PositionEKF ekf_filter;
    PositionUKF ukf_filter;
}

#endif // KALMANPOSITIONFILTERINGCONTROLLER_H
