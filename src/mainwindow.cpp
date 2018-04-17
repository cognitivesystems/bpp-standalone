#include <QDir>
#include <QtCore/QCoreApplication>
#include "mainwindow.h"

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags f) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  std::cout << "MainWindow constructor" << std::endl;

  MainWindow::singleton_ = this;

  ui->setupUi(this);

  view_ = new Qt3DExtras::Qt3DWindow();

  scene_3d_ = new SceneRenderer3D();

  // camera
  camera_ = view_->camera();
  resetCamera();

  Qt3DCore::QEntity* lightEntity = new Qt3DCore::QEntity(scene_3d_->getScene());
  Qt3DRender::QPointLight* light = new Qt3DRender::QPointLight(lightEntity);
  light->setColor("white");
  light->setIntensity(1);
  lightEntity->addComponent(light);
  Qt3DCore::QTransform* lightTransform = new Qt3DCore::QTransform(lightEntity);
  lightTransform->setTranslation(camera_->position());
  lightEntity->addComponent(lightTransform);

  // manipulator
  manipulator_ = new Qt3DExtras::QFirstPersonCameraController(scene_3d_->getScene());
  manipulator_->setLinearSpeed(50.f);
  manipulator_->setLookSpeed(180.f);
  manipulator_->setCamera(camera_);

  view_->setRootEntity(scene_3d_->getScene());

  QWidget* widget = createWindowContainer(view_, this);
  widget->resize(640, 480);
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
  camera_->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
  camera_->setUpVector(QVector3D(0.0f, 0.0f, 1.0f));
  camera_->setPosition(QVector3D(5.0f, 5.0f, 5.0f));
  camera_->setViewCenter(QVector3D(0, 0, 0));
}

void MainWindow::resetBoxes()
{
  if (!boxes_.empty())
  {
    boxes_.clear();
  }
  if (!planned_boxes_.empty())
  {
    planned_boxes_.clear();
  }
}

void MainWindow::resetScene()
{
  scene_3d_->clearScene();
}

void MainWindow::on_resetButton_clicked()
{
  qDebug() << "resetting Camera";
  resetCamera();
}

void MainWindow::on_loadButton_clicked()
{
  resetBoxes();
  resetScene();

  std::cout << "Loading boxes" << std::endl;
  QString boxes_file = ":/data/boxes.json";

  boxes_ = box_factory::BoxJsonParser::getBoxesFromJsonFile(boxes_file);
  std::cout << "Number of boxes " << boxes_.size() << std::endl;

  for (const bpa::Box& b : boxes_)
  {
    scene_3d_->addBoxEntity(b);
  }
}

void MainWindow::on_planButton_clicked()
{
  std::cout << "Planning" << std::endl;

  for (bpa::Box& b : boxes_)
  {
    b.position.position[0] -= b.m_length / 2;
    b.position.position[1] -= b.m_width / 2;
    b.position.position[2] -= b.m_height / 2;
  }

  planned_boxes_ = bpp_inf_.binPackingBoxes(boxes_);

  for (bpa::Box& b : planned_boxes_)
  {
    b.position.position[0] += b.m_length / 2;
    b.position.position[1] += b.m_width / 2;
    b.position.position[2] += b.m_height / 2;

    scene_3d_->updateBoxEntity(b);
  }
}

void MainWindow::on_estimateButton_clicked()
{
  std::cout << "Estimating parameters" << std::endl;

  ParamEstimator est;
  est.initialize();

  if (boxes_.size() <= 0)
  {
    std::cout << "Number of Boxes = 0. Estimator exiting!";
  }
  else
  {
    std::cout << "Setting estimator with boxes --> " << boxes_.size() << std::endl;
    est.setBoxData(boxes_);
    est.run();
  }
}

void MainWindow::on_deleteButton_clicked()
{
  std::cout << "Deleting model" << std::endl;

  resetBoxes();
  resetScene();
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  std::cout << "Closing main gui" << std::endl;
  killTimer(this->timer_id_);

  close();
}

void MainWindow::timerEvent(QTimerEvent* timerEvent)
{
}
