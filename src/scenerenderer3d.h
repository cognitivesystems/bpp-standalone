#ifndef SCENERENDERER3D_H
#define SCENERENDERER3D_H

#include <QString>
#include <QSharedPointer>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DCore/QAspectEngine>

#include <Qt3DCore/QComponent>
#include <Qt3DRender/qrenderaspect.h>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <QWidget>

#include <iostream>

typedef std::vector<QMatrix4x4> Transforms;

struct ObectPose{
    QVector3D position;
    QQuaternion orientation;
};

struct ObjectBody {
    QString body_name;
    QString mesh_url;
    ObectPose pose;
    QVector3D bbox;
    bool is_box;
};

struct ObjectModel {
    QString object_name;
    std::vector<ObjectBody > bodies;
};

struct ObjectComponents{
    std::vector<Qt3DCore::QComponent* > components;
};

typedef std::vector< ObjectModel > ObjectModels;

class SceneRenderer3D : public QWidget
{
    Q_OBJECT

public:
    SceneRenderer3D(QWidget *parent = 0);

    void addObjectBody(const ObjectBody& info);
    void addObjectModel(const ObjectModel& info);

    void removeObjectModel(const ObjectModel& model);

    void updateObjectModel(const ObjectModel &info);

    ObjectModels getModels();

    void deleteScene();
    void clearScene();

    void createTestScene();

    Qt3DCore::QEntity* getScene();

public slots:

    void slotAddObjectModel(const ObjectModel& model);

    void slotRemoveObjectModel(const ObjectModel& model);

    void slotUpdateObjectModel(const ObjectModel& model);

    void slotUpdateObjectModels(const std::vector<ObjectModel>& models);

private:

    std::string instance_name_;

    ObjectModels current_scene_models_;

    QMap<QString, Transforms > model_transform_map_;

    QMap<QString, Qt3DCore::QEntity* > model_entity_map_;

    QMap<QString, ObjectComponents > model_component_map_;

    Qt3DCore::QEntity* root_;

};

#endif // SCENERENDERER3D_H
