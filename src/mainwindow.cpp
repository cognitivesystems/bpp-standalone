#include <QDir>
#include <QtCore/QCoreApplication>
#include "mainwindow.h"

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags f)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow)
  , boxSet_(new BoxSet(this))
  , binPackingView_(*new BinPackingView(this))
  , binPackingViewMgr_(new BinPackingViewMgr(this, *boxSet_, binPackingView_))
  , particleFilter_(new filter::ParticleFilter(this))
  , distributionView_(*new DistributionView(this))
  , distributionViewMgr_(new DistributionViewMgr(this, *particleFilter_, distributionView_))
{
  std::cout << "MainWindow constructor" << std::endl;

  MainWindow::singleton_ = this;

  ui->setupUi(this);

  view_ = new Qt3DExtras::Qt3DWindow();

  // camera
  camera_ = view_->camera();
  resetCamera();

  Qt3DCore::QEntity* lightEntity = new Qt3DCore::QEntity(binPackingView_.getScene());
  Qt3DRender::QPointLight* light = new Qt3DRender::QPointLight(lightEntity);
  light->setColor("white");
  light->setIntensity(1);
  lightEntity->addComponent(light);
  Qt3DCore::QTransform* lightTransform = new Qt3DCore::QTransform(lightEntity);
  lightTransform->setTranslation(camera_->position());
  lightTransform->setTranslation(QVector3D(0, 0, 5.0));

  lightEntity->addComponent(lightTransform);

  // manipulator
  manipulator_ = new Qt3DExtras::QOrbitCameraController(binPackingView_.getScene());
  manipulator_->setLinearSpeed(50.f);
  manipulator_->setLookSpeed(180.f);
  manipulator_->setCamera(camera_);

  view_->setRootEntity(binPackingView_.getScene());

  QWidget* widget = createWindowContainer(view_, this);
  widget->resize(640, 480);
  ui->verticalLayout->addWidget(widget);

  this->timer_id_ = startTimer(1);

  window.setCentralWidget(distributionView_.getChartView());
  window.resize(400, 300);
  window.show();

  std::string url = "/home/nair/workspace/bpp_code/bpp-standalone/build/mesh.obj";
  binPackingView_.addObjEntity(url);
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
  boxSet_->removeAllBoxes();
}

void MainWindow::on_resetButton_clicked()
{
  qDebug() << "resetting Camera";
  resetCamera();
}

void MainWindow::on_loadButton_clicked()
{
  resetBoxes();

  std::cout << "Loading boxes" << std::endl;
  QString boxes_file = ":/data/boxes.json";

  boxSet_->loadBoxesFromFile(boxes_file);

  std::cout << "Number of boxes " << boxSet_->getBoxes().size() << std::endl;
}

void MainWindow::on_planButton_clicked()
{
  std::cout << "Planning" << std::endl;

  doBinPacking();
}

void MainWindow::on_estimateButton_clicked()
{
  std::cout << "Estimating parameters" << std::endl;

  vector<bpa::Box> boxes = boxSet_->getBoxes();

  ParamEstimator est(*particleFilter_);
  est.initialize();

  if (boxes.size() <= 0)
  {
    std::cout << "Number of Boxes = 0. Estimator exiting!";
  }
  else
  {
    std::cout << "Setting estimator with boxes --> " << boxes.size() << std::endl;
    est.setBoxData(boxes);

    while (true)
    {
      est.run();

      std::vector<float> avg_est = est.getAvg();

      std::cout << "Avg pose --> ";
      for (size_t i = 0; i < avg_est.size(); ++i)
      {
      std:;
        cout << avg_est[i] << " ";
      }
      std::cout << "\n";

      std::shared_ptr<bpa::Params> paramsPtr(new bpa::Params());
      double helt_rate = 0.95;
      double w_supported = 0.1;
      double neightbour_constant = 0.0;
      double w_assignment = 0.3;
      double w_place_near = 0.3;
      double bin_height = 0.02;
      double min_box_size = 0.3;
      double w_item_in_the_bottom_area = 0.3;
      double w_high_items_good_placed = 0.0;
      bool generate_simulated_boxes = false;
      bool start_with_all_edges_as_fp = false;
      int search_height = 10;
      int search_width = 10;

      paramsPtr.get()->setAll(avg_est[0], avg_est[1], avg_est[2], avg_est[3], helt_rate, w_supported, avg_est[4],
                              neightbour_constant, w_assignment, w_place_near, bin_height, min_box_size,
                              w_item_in_the_bottom_area, w_high_items_good_placed, generate_simulated_boxes,
                              start_with_all_edges_as_fp, search_height, search_width);

      doBinPacking(paramsPtr);
      QApplication::processEvents();
    }
  }
}

void MainWindow::on_deleteButton_clicked()
{
  std::cout << "Deleting model" << std::endl;
  resetBoxes();
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

void MainWindow::doBinPacking()
{
  std::vector<bpa::Box> boxes = boxSet_->getBoxes();

  for (bpa::Box& b : boxes)
  {
    b.position.position[0] -= b.m_length / 2;
    b.position.position[1] -= b.m_width / 2;
    b.position.position[2] -= b.m_height / 2;
  }

  std::vector<bpa::Box> planned_boxes = bpp_inf_.binPackingBoxes(boxes);

  for (bpa::Box& b : planned_boxes)
  {
    b.position.position[0] += b.m_length / 2;
    b.position.position[1] += b.m_width / 2;
    b.position.position[2] += b.m_height / 2;
  }

  boxSet_->updateBoxes(planned_boxes);
}

void MainWindow::doBinPacking(std::shared_ptr<bpa::Params>& params)
{
  std::vector<bpa::Box> boxes = boxSet_->getBoxes();

  for (bpa::Box& b : boxes)
  {
    b.position.position[0] -= b.m_length / 2;
    b.position.position[1] -= b.m_width / 2;
    b.position.position[2] -= b.m_height / 2;
  }

  bpp_inf_.setParams(params);

  std::vector<bpa::Box> planned_boxes = bpp_inf_.binPackingBoxes(boxes);

  for (bpa::Box& b : planned_boxes)
  {
    b.position.position[0] += b.m_length / 2;
    b.position.position[1] += b.m_width / 2;
    b.position.position[2] += b.m_height / 2;
  }

  boxSet_->updateBoxes(planned_boxes);
}
