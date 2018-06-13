#include "views/base/xdaxisbody.h"

#include <QWidget>
#include <QGridLayout>
#include <Qt3DCore/QEntity>
#include <Qt3DRender/QMaterial>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QCameraLens>
#include <Qt3DExtras>

const double XDAxisBody::body_width { 5 };
const double XDAxisBody::body_length { 10 };
const double XDAxisBody::body_height { 1 };
const double XDAxisBody::sphere_radius { 0.5 };

XDAxisBody::XDAxisBody(QWidget *dummy_plot, QGridLayout *container_layout)
    : dummy_plot{ dummy_plot }, container_layout{ container_layout }, brought_up{ false }
{
    plot.defaultFrameGraph()->setClearColor(QColor(126, 192, 238));

    orient_plot_container = QWidget::createWindowContainer(&plot);

    Qt3DCore::QEntity *root = new Qt3DCore::QEntity;
    Qt3DExtras::QPhongMaterial * material = new Qt3DExtras::QPhongMaterial(root);
    material->setDiffuse(Qt::red);

    Qt3DCore::QEntity * body = new Qt3DCore::QEntity(root);
    Qt3DExtras::QCuboidMesh * mesh = new Qt3DExtras::QCuboidMesh;

    mesh->setXExtent(body_width);
    mesh->setYExtent(body_length);
    mesh->setZExtent(body_height);

    body_transform = new Qt3DCore::QTransform;

    body->addComponent(mesh);
    body->addComponent(body_transform);
    body->addComponent(material);

    Qt3DExtras::QPhongMaterial * plane_material = new Qt3DExtras::QPhongMaterial(root);
    plane_material->setDiffuse(QColor(77, 158, 58));
    plane_material->setSpecular(Qt::white);

    Qt3DCore::QEntity * reference = new Qt3DCore::QEntity(root);
    Qt3DExtras::QPlaneMesh * plane = new Qt3DExtras::QPlaneMesh;

    plane->setHeight(200);
    plane->setWidth(200);

    Qt3DCore::QTransform * plane_transform = new Qt3DCore::QTransform;
    plane_transform->setRotation(QQuaternion::fromAxisAndAngle(1, 0, 0, 90));
    plane_transform->setTranslation(QVector3D(0, 0, -6));

    reference->addComponent(plane);
    reference->addComponent(plane_material);
    reference->addComponent(plane_transform);

    reference->setEnabled(true);

    Qt3DCore::QEntity * north_entity = new Qt3DCore::QEntity(root);
    Qt3DExtras::QSphereMesh * sphere = new Qt3DExtras::QSphereMesh;

    Qt3DExtras::QPhongMaterial * sphere_material = new Qt3DExtras::QPhongMaterial(root);
    sphere_material->setDiffuse(Qt::blue);

    sphere_transform = new Qt3DCore::QTransform;

    sphere_transform->setTranslation(QVector3D(0, body_length / 2, body_height / 2));

    sphere->setRadius(sphere_radius);

    north_entity->addComponent(sphere_transform);
    north_entity->addComponent(sphere);
    north_entity->addComponent(sphere_material);

    north_entity->setEnabled(true);

    //
    Qt3DCore::QEntity * north_line = new  Qt3DCore::QEntity(root);
    Qt3DExtras::QCylinderMesh * nline = new Qt3DExtras::QCylinderMesh;

    Qt3DCore::QTransform * nline_transform = new Qt3DCore::QTransform;
    nline_transform->setTranslation(QVector3D(0, 50, -6));
    //

    nline->setLength(100);
    nline->setRadius(0.2);

    north_line->addComponent(nline_transform);
    north_line->addComponent(nline);
    north_line->addComponent(sphere_material);

    north_line->setEnabled(true);

    //
    Qt3DCore::QEntity * south_line = new  Qt3DCore::QEntity(root);
    Qt3DExtras::QCylinderMesh * sline = new Qt3DExtras::QCylinderMesh;

    Qt3DCore::QTransform * sline_transform = new Qt3DCore::QTransform;
    sline_transform->setTranslation(QVector3D(0, -50, -6));
    //

    sline->setLength(100);
    sline->setRadius(0.2);

    south_line->addComponent(sline_transform);
    south_line->addComponent(sline);
    south_line->addComponent(material);

    south_line->setEnabled(true);

    //
    Qt3DCore::QEntity * light_entity = new Qt3DCore::QEntity(root);
    Qt3DRender::QPointLight * light = new Qt3DRender::QPointLight(light_entity);
    light->setColor(Qt::white);
    light->setIntensity(1.);

    Qt3DCore::QTransform * light_transform = new Qt3DCore::QTransform;
    light_transform->setTranslation(QVector3D(0, -20, 20));

    light_entity->addComponent(light_transform);
    light_entity->addComponent(light);

    light_entity->setEnabled(true);

    Qt3DRender::QCamera * camera = plot.camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setViewCenter(QVector3D(0, 0, 0));
    camera->setPosition(QVector3D(0, 0, 20.0f));
    camera->rotateAboutViewCenter(QQuaternion::fromAxisAndAngle(1, 0, 0, 79));

    Qt3DExtras::QOrbitCameraController * camController = new Qt3DExtras::QOrbitCameraController(root);
    camController->setLinearSpeed( 50.0f );
    camController->setLookSpeed( 180.0f );
    camController->setCamera(camera);

    plot.setRootEntity(root);
}

void XDAxisBody::bring_up()
{
    container_layout->replaceWidget(dummy_plot, orient_plot_container);
    plot.show();
    brought_up = true;
}

bool XDAxisBody::is_brought_up()
{
    return brought_up;
}
