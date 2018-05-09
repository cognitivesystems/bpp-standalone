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
    light->setIntensity(0.5);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform* lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(camera_->position());
    lightTransform->setTranslation(QVector3D(0,0,10.0));

    lightEntity->addComponent(lightTransform);

    // manipulator
    manipulator_ = new Qt3DExtras::QOrbitCameraController(scene_3d_->getScene());
    manipulator_->setLinearSpeed(50.f);
    manipulator_->setLookSpeed(180.f);
    manipulator_->setCamera(camera_);

    view_->setRootEntity(scene_3d_->getScene());

    QWidget* widget = createWindowContainer(view_, this);
    widget->resize(640, 480);
    ui->verticalLayout->addWidget(widget);

    this->timer_id_ = startTimer(1);

    x_target=linspace(0.0,1.0, 100);
    y_target=linspace(0.0,1.0, 100);

    series0 = new QLineSeries();
    series1 = new QLineSeries();
    series2 = new QLineSeries();
    series3 = new QLineSeries();
    series4 = new QLineSeries();

    chart = new QChart();
    chart->legend()->hide();
    chart->addSeries(series0);
    chart->addSeries(series1);
    chart->addSeries(series2);
    chart->addSeries(series3);
    chart->addSeries(series4);
    chart->createDefaultAxes();
    chart->axisY()->setRange(0, 20);
    chart->setTitle("Parameter 0");
    chart->show();

    chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    window.setCentralWidget(chartView);
    window.resize(400, 300);
    window.show();

    bpa::Box floor;
    floor.m_name="floor";
    floor.position.position[0]=0.0;
    floor.position.position[1]=0.0;
    floor.position.position[2]=0.0;
    floor.rotation=0.0;
    floor.m_length=25.00;
    floor.m_width=25.00;
    floor.m_height=0.05;
    floor.m_type="floor";
    scene_3d_->addBoxEntity(floor);

    bpa::Box pallet;
    pallet.m_name="pallet";
    pallet.m_type="pallet";
    pallet.position.position[0]=0.0;
    pallet.position.position[1]=0.0;
    pallet.position.position[2]=0.0;
    pallet.rotation=0.0;
    pallet.m_length=2.28;
    pallet.m_width=3.00;
    pallet.m_height=0.05;
    pallet.position.position[0] += pallet.m_length / 2;
    pallet.position.position[1] += pallet.m_width / 2;
    pallet.position.position[2] += pallet.m_height / 2;

    scene_3d_->addBoxEntity(pallet);

    //    std::string url="/home/nair/workspace/bpp_code/bpp-standalone/build/mesh.obj";
    //    scene_3d_->addObjEntity(url);
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

void MainWindow::clearBoxes()
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

void MainWindow::clearScene()
{
    scene_3d_->clearScene();
}

void MainWindow::resetScene()
{    std::vector<bpa::Box> boxes=boxes_;

     for (bpa::Box& b : boxes)
     {
         scene_3d_->updateBoxEntity(b);
     }
}

void MainWindow::on_resetButton_clicked()
{
    qDebug() << "resetting Camera";
    resetCamera();
}

void MainWindow::on_loadButton_clicked()
{
    clearBoxes();
    //    resetScene();

    std::cout << "Loading boxes" << std::endl;
    QString boxes_file = ":/data/boxes.json";

    boxes_ = box_factory::BoxJsonParser::getBoxesFromJsonFile(boxes_file);
    std::cout << "Number of boxes " << boxes_.size() << std::endl;

    for (bpa::Box& b : boxes_)
    {
        b.position.position[0] += 4.0;

        //        std::cout << "Small box --> " << b.m_name << " " << b.m_length << " " << b.m_width << " " << b.m_height << std::endl;
        //        std::cout << "Small box pose --> " << b.position.position[0] << " " << b.position.position[1] << " " << b.position.position[2] << std::endl;

        if(b.m_length==0.3 || b.m_width==0.3){
            std::cout << "Small box --> " << b.m_length << " " << b.m_width << " " << b.m_height << std::endl;
            std::cout << "Small box pose --> " << b.position.position[0] << " " << b.position.position[1] << " " << b.position.position[2] << std::endl;

        }

        b.rotation=30.0;
        scene_3d_->addBoxEntity(b);

    }
}

void MainWindow::on_planButton_clicked()
{
    std::cout << "Planning" << std::endl;

    doBinPacking();

}

void MainWindow::on_estimateButton_clicked()
{
    std::cout << "Estimating parameters" << std::endl;

    ParamEstimator est;
    est.initialize();


    QObject::connect(&est, SIGNAL(send_reset_scene()),
                     this, SLOT(slot_reset_scene()));
    QObject::connect(&est, SIGNAL(send_update_boxes(Boxes)),
                     this, SLOT(slot_update_boxes(Boxes)));


    if (boxes_.size() <= 0)
    {
        std::cout << "Number of Boxes = 0. Estimator exiting!";
    }
    else
    {
        std::cout << "Setting estimator with boxes --> " << boxes_.size() << std::endl;
        est.setBoxData(boxes_);

        while(true){

            est.run();

            std::vector<float > alpha=est.getAlpha();
            std::vector<float > beta=est.getBeta();

            pdf0=est.computePDF((double)(alpha[0]), (double)(beta[0]), x_target);
            pdf1=est.computePDF((double)(alpha[1]), (double)(beta[1]), x_target);
            pdf2=est.computePDF((double)(alpha[2]), (double)(beta[2]), x_target);
            pdf3=est.computePDF((double)(alpha[3]), (double)(beta[3]), x_target);
            pdf4=est.computePDF((double)(alpha[4]), (double)(beta[4]), x_target);

            //            std::cout << "Param 0 --> " << alpha[0] << " " << beta[0] << std::endl;
            //            std::cout << "Param 1 --> " << alpha[1] << " " << beta[1] << std::endl;
            //            std::cout << "Param 2 --> " << alpha[2] << " " << beta[2] << std::endl;
            //            std::cout << "Param 3 --> " << alpha[3] << " " << beta[3] << std::endl;
            //            std::cout << "Param 4 --> " << alpha[4] << " " << beta[4] << std::endl;

            series0->clear();
            series1->clear();
            series2->clear();
            series3->clear();
            series4->clear();

            for(size_t i=0;i<pdf0.size();++i){
                *series0<<QPointF(x_target[i], pdf0[i]);
                *series1<<QPointF(x_target[i], pdf1[i]);
                *series2<<QPointF(x_target[i], pdf2[i]);
                *series3<<QPointF(x_target[i], pdf3[i]);
                *series4<<QPointF(x_target[i], pdf4[i]);
                //            std::cout << pdf0[i] << " " << x_target[i] << std::endl;
            }

            window.update();
            QApplication::processEvents();

            std::vector<float > avg_est=est.getAvg();

            std::cout << "Avg pose --> " ;
            for(size_t i=0;i<avg_est.size();++i){
                std::cout << avg_est[i] << " ";
            }
            std::cout << "\n";

            std::shared_ptr<bpa::Params> paramsPtr(new bpa::Params());
            double helt_rate=0.95;
            double w_supported=0.1;
            double neightbour_constant=0.0;
            double w_assignment=0.3;
            double w_place_near=0.3;
            double bin_height=0.02;
            double min_box_size=0.3;
            double w_item_in_the_bottom_area=0.3;
            double w_high_items_good_placed=0.0;
            bool generate_simulated_boxes=false;
            bool start_with_all_edges_as_fp=false;
            int search_height=10;
            int search_width=10;

            paramsPtr.get()->setAll(avg_est[0], avg_est[1], avg_est[2], avg_est[3], helt_rate, w_supported, avg_est[4],
                    neightbour_constant, w_assignment, w_place_near, bin_height, min_box_size, w_item_in_the_bottom_area, w_high_items_good_placed,
                    generate_simulated_boxes, start_with_all_edges_as_fp, search_height, search_width);


            //doBinPacking(paramsPtr);
            //QApplication::processEvents();
        }

    }
}

void MainWindow::on_deleteButton_clicked()
{
    std::cout << "Deleting model" << std::endl;

    clearBoxes();
    clearScene();
}

void MainWindow::on_genButton_clicked()
{

    QObject::connect(ui->sizeSlider, SIGNAL(valueChanged(int)),
                     this, SLOT(boxSizeUpdate(int)));

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_l(0, 2.28);
    std::uniform_real_distribution<> dis_w(0, 3.0);
    std::uniform_real_distribution<> dis_h(0, 3.0);

    boxes_.clear();

    bpa::Box b;
    b.m_type="box";
    b.m_length=0.01;
    b.m_width=0.01;
    b.m_height=0.01;

    for(size_t i=0;i<100;++i){
        b.m_name=QString::number(i).toStdString();
        b.position.position[0]=dis_l(gen);
        b.position.position[1]=dis_w(gen);
        b.position.position[2]=dis_h(gen);
        b.rotation=0.0;

        //        std::cout << "Small box --> " << b.m_name << " " << b.m_length << " " << b.m_width << " " << b.m_height << std::endl;
        //        std::cout << "Small box pose --> " << b.position.position[0] << " " << b.position.position[1] << " " << b.position.position[2] << std::endl;

        boxes_.push_back(b);
        scene_3d_->addBoxEntity(b);
    }
}

void MainWindow::slot_reset_scene()
{
    resetScene();
}

void MainWindow::slot_update_boxes(const Boxes &bxs)
{
    std::cout << "Updating Boxes +++++++++++++++++++++++++++++++= " << std::endl;
    for (bpa::Box b : bxs)
    {
        scene_3d_->updateBoxEntity(b);
    }
    QMainWindow::update();
    QApplication::processEvents();

}

void MainWindow::boxSizeUpdate(int value)
{
    std::cout << "Updating Boxes Sizes +++++++++++++++++++++++++++++++= " << std::endl;

    std::vector<bpa::Box> boxes=boxes_;

    for (bpa::Box b : boxes)
    {
        b.m_length=0.01*value;
        b.m_width=0.01*value;
        b.m_height=0.01*value;

        scene_3d_->updateBoxEntity(b);
    }
    QMainWindow::update();
    QApplication::processEvents();
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
    std::vector<bpa::Box> boxes=boxes_;

    for (bpa::Box& b : boxes)
    {
        b.position.position[0] -= b.m_length / 2;
        b.position.position[1] -= b.m_width / 2;
        b.position.position[2] -= b.m_height / 2;
        scene_3d_->updateBoxEntity(b);

    }

    planned_boxes_ = bpp_inf_.binPackingBoxes(boxes);

    for (bpa::Box& b : planned_boxes_)
    {
        b.position.position[0] += b.m_length / 2;
        b.position.position[1] += b.m_width / 2;
        b.position.position[2] += b.m_height / 2;

        scene_3d_->updateBoxEntity(b);
    }
}

void MainWindow::doBinPacking(std::shared_ptr<bpa::Params> &params)
{
    std::vector<bpa::Box> boxes=boxes_;

    for (bpa::Box& b : boxes)
    {
        b.position.position[0] -= b.m_length / 2;
        b.position.position[1] -= b.m_width / 2;
        b.position.position[2] -= b.m_height / 2;
        scene_3d_->updateBoxEntity(b);

    }

    bpp_inf_.setParams(params);

    planned_boxes_ = bpp_inf_.binPackingBoxes(boxes);

    for (bpa::Box& b : planned_boxes_)
    {
        b.position.position[0] += b.m_length / 2;
        b.position.position[1] += b.m_width / 2;
        b.position.position[2] += b.m_height / 2;

        scene_3d_->updateBoxEntity(b);
    }
    QMainWindow::update();
    QApplication::processEvents();
}
