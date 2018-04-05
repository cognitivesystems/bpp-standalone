#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent, Qt::WindowFlags f) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    std::cout << "MainWindow constructor" << std::endl;

    MainWindow::singleton_ = this;

    ui->setupUi(this);

    view = new Qt3DExtras::Qt3DWindow();

    scene_3d_=new SceneRenderer3D();

    // camera
    camera = view->camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setPosition(QVector3D(0, 0, 40.0f));
    camera->setViewCenter(QVector3D(0, 0, 0));

    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(scene_3d_->getScene());
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(1);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(camera->position());
    lightEntity->addComponent(lightTransform);


    // manipulator
    manipulator = new Qt3DExtras::QOrbitCameraController(scene_3d_->getScene());
    manipulator->setLinearSpeed(50.f);
    manipulator->setLookSpeed(180.f);
    manipulator->setCamera(camera);

    view->setRootEntity(scene_3d_->getScene());

    QWidget* widget=createWindowContainer(view, this);
    widget->resize(640,480);
    ui->verticalLayout->addWidget(widget);
    this->timer_id_ = startTimer(1);

}

MainWindow::~MainWindow()
{
    delete ui;
}

MainWindow* MainWindow::instance()
{
    if (NULL == MainWindow::singleton_)
    {
        new MainWindow();
    }
    return MainWindow::singleton_;
}

void MainWindow::resetCamera()
{
    // camera
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setPosition(QVector3D(0, 0, 40.0f));
    camera->setViewCenter(QVector3D(0, 0, 0));

    manipulator->setCamera(camera);

}

void MainWindow::on_resetButton_clicked(){
    qDebug() << "resetting Camera";
    resetCamera();
}

void MainWindow::on_deleteButton_clicked()
{
    std::cout << "Deleting model" << std::endl;

    scene_3d_->removeObjectModel(models.front());

}


void MainWindow::closeEvent(QCloseEvent *event)
{
    std::cout << "Closing main gui" << std::endl;
    killTimer(this->timer_id_);

    close();

}

void MainWindow::timerEvent(QTimerEvent* timerEvent)
{    
    static bool init= false;
    if(!init){
        std::cout << "Adding object model" << std::endl;
        ObjectModel model;
        model.object_name="box_0";
        ObjectBody body;
        body.body_name="box_0";
        body.pose.position.setX(0);
        body.pose.position.setY(0);
        body.pose.position.setZ(0);
        body.pose.orientation.setX(0);
        body.pose.orientation.setY(0);
        body.pose.orientation.setZ(0);
        body.pose.orientation.setScalar(1);
        body.is_box=true;
        body.bbox.setX(1);
        body.bbox.setY(2);
        body.bbox.setZ(3);

        model.bodies.push_back(body);

        models.push_back(model);
        scene_3d_->addObjectModel(model);
        init=true;
    }
}
