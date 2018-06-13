#ifndef XDAXISBODY_H
#define XDAXISBODY_H

#include <Qt3DCore/QTransform>
#include <Qt3DExtras/Qt3DWindow>

#include <memory>

class QWidget;
class QGridLayout;

struct XDAxisBody
{
    XDAxisBody(QWidget * dummy_plot, QGridLayout * container_layout);

    void bring_up();
    bool is_brought_up();

private:
    Qt3DExtras::Qt3DWindow plot;
    Qt3DCore::QTransform * body_transform;
    Qt3DCore::QTransform * sphere_transform;

    QWidget * dummy_plot;
    QWidget * orient_plot_container;
    QGridLayout * container_layout;
    bool brought_up;

    static const double body_width;
    static const double body_length;
    static const double body_height;
    static const double sphere_radius;
};

#endif // XDAXISBODY_H
