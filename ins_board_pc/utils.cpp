/*
 * utils.cpp
 *
 *      Author: Ermakov_P
 */
#include "utils.h"
#include "packets.h"
#include "qcustomplot.h"

#include <cmath>
#include <limits>

#include <QString>
#include <QDateTime>

namespace utils
{

//! Multiplier to convert speed units from m/s to knots.
static const double ms2knots { 1.94384 };

//! How many degrees in PI radians.
static const double degrees_per_pi { 180.0 };

//! Multiplier to convert floating-point value into 16.16 fixed-point value.
static const int16_t fixed16d16_mult { std::numeric_limits<int16_t>::max() };

double radians_to_degrees(double radians)
{
    return radians / M_PI * degrees_per_pi;
}

double degrees_to_radians(double degrees)
{
    return degrees / degrees_per_pi * M_PI;
}

double fix_angle(double radians)
{
	if(radians >= M_PI)
	{
		return radians - 2 * M_PI;
	}
	else if(radians < - M_PI)
	{
		return radians + 2 * M_PI;
	}
	else
	{
		return radians;
	}
}

int32_t float_to_fixed(float val)
{
    return static_cast<int32_t>(val * fixed16d16_mult);
}

int32_t double_to_fixed(double val)
{
    return static_cast<int32_t>(val * fixed16d16_mult);
}

int32_t angle_to_fixed(double radians)
{
    return static_cast<int32_t>(radians * fixed16d16_mult / M_PI);
}

double ms_to_knots(double ms)
{
	return ms * ms2knots;
}

void update_3axis_plot(QCustomPlot * plot, const Vector3D & vec)
{
    int pts = plot->graph(0)->dataCount();
    if(pts < plot->xAxis->range().upper)
    {
        plot->graph(0)->addData(pts, vec[0]);
        plot->graph(1)->addData(pts, vec[1]);
        plot->graph(2)->addData(pts, vec[2]);
        plot->replot();
    }
    else
    {
        plot->graph(0)->data()->clear();
        plot->graph(1)->data()->clear();
        plot->graph(2)->data()->clear();
    }
}

void clear_3axis_plot(QCustomPlot * plot)
{
    plot->graph(0)->data().clear();
    plot->graph(1)->data().clear();
    plot->graph(2)->data().clear();
}

void update_track(QCustomPlot * plot, QCPCurve * track, const Vector3D & point)
{
    track->addData(point[0], point[1]);

    plot->rescaleAxes();

    double expected_y_span = plot->xAxis->range().size() * plot->axisRect()->height() / plot->axisRect()->width();

    if(plot->yAxis->range().size() < expected_y_span)
    {
        plot->yAxis->setScaleRatio(plot->xAxis, 1);
    }
    else
    {
        plot->xAxis->setScaleRatio(plot->yAxis, 1);
    }

    plot->replot();
}

QString double_view(double val, std::size_t digits)
{
    return QString::number(val, 'f', digits);
}

QString gps_time_string(const Timestamp & ts)
{
    QDateTime dt;
    dt.setDate(QDate(ts.year, ts.month, ts.day));
    dt.setTime(QTime(ts.hour, ts.minute, ts.second, ts.ssecond * 10));
    return dt.toString();
}

}
