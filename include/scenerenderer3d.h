#ifndef SCENERENDERER3D_H
#define SCENERENDERER3D_H

//#include <QSharedPointer>
#include <QObject>
#include <QString>
#include <Qt3DCore/QAspectEngine>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>

#include <QWidget>
#include <Qt3DCore/QComponent>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DRender/QMesh>
#include <Qt3DRender/qrenderaspect.h>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DRender/QObjectPicker>
#include <Qt3DRender/QPickEvent>

#include <iostream>
#include "objentity.h"
#include "boxentity.h"
#include "Box.h"

class SceneRenderer3D : public QWidget
{
    Q_OBJECT

public:
    SceneRenderer3D(QWidget* parent = 0);

    void addObjEntity(const std::string obj_url);

    void addBoxEntity(const bpa::Box& box);
    void updateBoxEntity(const bpa::Box& box);
    void removeBoxEntity(const bpa::Box& box);
    void removeAllBoxEntities();

    void deleteScene();
    void clearScene();
    void createTestScene();

    Qt3DCore::QEntity* getScene();

public slots:
    void onPicked(Qt3DRender::QPickEvent *evt);

public slots:
    void slotAddBoxEntity(const bpa::Box& box);
    void slotUpdateBoxEntity(const bpa::Box& box);
    void slotRemoveBoxEntity(const bpa::Box& box);
    void slotUpdateBoxEntities(const std::vector<bpa::Box>& boxes);

private:
    std::string instance_name_;
    Qt3DCore::QEntity* root_;
    QMap<QString, BoxEntity*> uuid_entity_map_;
    Qt3DRender::QObjectPicker *picker1;
};

#endif  // SCENERENDERER3D_H
