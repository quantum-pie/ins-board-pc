#ifndef KALMANORIENTATIONFILTERINGCONTROLLER_H
#define KALMANORIENTATIONFILTERINGCONTROLLER_H

#include "filtering/filters/generickalman.h"

#include <QObject>

class FilterInput;

class KalmanOrientationFilteringController : public QObject
{
    Q_OBJECT

public:
    KalmanOrientationFilteringController(QObject * parent = nullptr);

public slots:
    void handle_start(bool en);
    void handle_strategy(int idx);
    void handle_input(const FilterInput & z);

private:
    OrientationEKF ekf_filter;
    OrientationUKF ukf_filter;
}

#endif // KALMANORIENTATIONFILTERINGCONTROLLER_H
