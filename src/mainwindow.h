#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QPointLight>
#include <Qt3DExtras/QOrbitCameraController>

#include <iostream>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "scenerenderer3d.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    virtual ~MainWindow();
    static MainWindow* instance();

    void resetCamera();

protected:
    MainWindow(QWidget* parent = NULL, Qt::WindowFlags f = 0);

    void closeEvent(QCloseEvent *event);

    virtual void timerEvent(QTimerEvent* timerEvent);

private slots:
    void on_resetButton_clicked();
    void on_deleteButton_clicked();


private:
    Ui::MainWindow *ui;

    static MainWindow* singleton_;

    Qt3DExtras::Qt3DWindow* view_;
    Qt3DRender::QCamera *camera_;
    Qt3DExtras::QOrbitCameraController* manipulator_;

    SceneRenderer3D* scene_3d_;

    std::vector<ObjectModel > models_;
    int timer_id_;
};

#endif // MAINWINDOW_H
