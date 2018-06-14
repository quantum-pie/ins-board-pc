#ifndef XDORIENTATIONVIEW_H
#define XDORIENTATIONVIEW_H

#include "views/base/IBaseView.h"
#include "core/IOrientationProvider.h"

#include <Qt3DCore/QTransform>
#include <Qt3DExtras/Qt3DWindow>

class QWidget;
class QGridLayout;
class QQuaternion;

struct XDOrientationView : IOrientationView
{
    XDOrientationView(QWidget * dummy_plot, QGridLayout * container_layout);
    ~XDOrientationView() override = default;

    void update(const ViewModel & pvd) override;
    void clear() override;

private:
    void bring_up();
    bool is_brought_up();
    void apply_rotation(const QQuaternion & q);

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

#endif // XDORIENTATIONVIEW_H
