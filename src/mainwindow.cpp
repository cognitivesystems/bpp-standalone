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

    camera_->setPosition(QVector3D(0, 5, 1));

    Qt3DCore::QEntity* lightEntity = new Qt3DCore::QEntity(scene_3d_->getScene());
    Qt3DRender::QPointLight* light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(0.5);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform* lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(QVector3D(0,0,6.0));
    lightEntity->addComponent(lightTransform);


    Qt3DCore::QEntity* lightEntity1 = new Qt3DCore::QEntity(scene_3d_->getScene());
    Qt3DRender::QPointLight* light1 = new Qt3DRender::QPointLight(lightEntity1);
    light1->setColor("white");
    light->setIntensity(0.5);
    lightEntity1->addComponent(light);
    Qt3DCore::QTransform* lightTransform1 = new Qt3DCore::QTransform(lightEntity1);
    lightTransform1->setTranslation(QVector3D(6,0,2.0));
    lightEntity1->addComponent(lightTransform1);


    Qt3DCore::QEntity* lightEntity2 = new Qt3DCore::QEntity(scene_3d_->getScene());
    Qt3DRender::QPointLight* light2 = new Qt3DRender::QPointLight(lightEntity2);
    light2->setColor("white");
    light->setIntensity(0.5);
    lightEntity2->addComponent(light2);
    Qt3DCore::QTransform* lightTransform2= new Qt3DCore::QTransform(lightEntity2);
    lightTransform2->setTranslation(QVector3D(0,6,2.0));
    lightEntity2->addComponent(lightTransform2);

    Qt3DCore::QEntity* lightEntity3 = new Qt3DCore::QEntity(scene_3d_->getScene());
    Qt3DRender::QPointLight* light3 = new Qt3DRender::QPointLight(lightEntity3);
    light3->setColor("white");
    light->setIntensity(0.5);
    lightEntity2->addComponent(light3);
    Qt3DCore::QTransform* lightTransform3 = new Qt3DCore::QTransform(lightEntity3);
    lightTransform3->setTranslation(QVector3D(-6,0,2.0));
    lightEntity3->addComponent(lightTransform3);

    Qt3DCore::QEntity* lightEntity4 = new Qt3DCore::QEntity(scene_3d_->getScene());
    Qt3DRender::QPointLight* light4 = new Qt3DRender::QPointLight(lightEntity4);
    light4->setColor("white");
    light->setIntensity(0.5);
    lightEntity4->addComponent(light4);
    Qt3DCore::QTransform* lightTransform4= new Qt3DCore::QTransform(lightEntity4);
    lightTransform4->setTranslation(QVector3D(0,6,2.0));
    lightEntity4->addComponent(lightTransform4);

    //    lightEntity->addComponent(lightTransform);
    //    lightEntity->addComponent(lightTransform1);
    //    lightEntity->addComponent(lightTransform2);
    //    lightEntity->addComponent(lightTransform3);
    //    lightEntity->addComponent(lightTransform4);

    // manipulator
    manipulator_ = new Qt3DExtras::QOrbitCameraController(scene_3d_->getScene());
    manipulator_->setLinearSpeed(50.f);
    manipulator_->setLookSpeed(180.f);
    manipulator_->setCamera(camera_);

    view_->setRootEntity(scene_3d_->getScene());

    QWidget* widget = createWindowContainer(view_, this);
    widget->resize(640, 480);
    ui->verticalLayout->addWidget(widget);

    //    this->timer_id_ = startTimer(1);

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
    floor.m_length=5.00;
    floor.m_width=5.00;
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
    pallet.m_length=2.44;
    pallet.m_width=3.18;
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
    QString boxes_file = "boxes.json";

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

void MainWindow::on_genButton_clicked(){
    boxes_.clear();

    bpa::Box b;
    b.m_type="pallet_face";
    b.m_length=0.01;
    b.m_width=3.0;
    b.m_height=3.0;
    b.m_name="0";

    b.position.position[0]=1.5;
    b.position.position[1]=0;
    b.position.position[2]=1.5;
    b.rotation=0.0;
    boxes_.push_back(b);
    boxes_map_[QString(b.m_name.c_str())]=b;
    //    scene_3d_->addBoxEntity(b);

    b.m_type="pallet_face";
    b.m_length=0.01;
    b.m_width=3.0;
    b.m_height=3.0;
    b.m_name="1";

    b.position.position[0]=-1.5;
    b.position.position[1]=0;
    b.position.position[2]=1.5;
    b.rotation=0.0;
    boxes_.push_back(b);
    boxes_map_[QString(b.m_name.c_str())]=b;
    //    scene_3d_->addBoxEntity(b);

    b.m_type="pallet_face";
    b.m_length=3.0;
    b.m_width=0.01;
    b.m_height=3.0;
    b.m_name="2";

    b.position.position[0]=0;
    b.position.position[1]=1.5;
    b.position.position[2]=1.5;
    b.rotation=0.0;
    boxes_.push_back(b);
    boxes_map_[QString(b.m_name.c_str())]=b;
    //    scene_3d_->addBoxEntity(b);


    b.m_type="pallet_face";
    b.m_length=3.0;
    b.m_width=0.01;
    b.m_height=3.0;
    b.m_name="3";

    b.position.position[0]=0;
    b.position.position[1]=-1.5;
    b.position.position[2]=1.5;
    b.rotation=0.0;
    boxes_.push_back(b);
    boxes_map_[QString(b.m_name.c_str())]=b;
    //    scene_3d_->addBoxEntity(b);

    b.m_type="pallet_face";
    b.m_length=3.0;
    b.m_width=3.0;
    b.m_height=0.01;
    b.m_name="4";

    b.position.position[0]=0;
    b.position.position[1]=0;
    b.position.position[2]=0;
    b.rotation=0.0;
    boxes_.push_back(b);
    boxes_map_[QString(b.m_name.c_str())]=b;
    //    scene_3d_->addBoxEntity(b);


    b.m_type="pallet_face";
    b.m_length=3.0;
    b.m_width=3.0;
    b.m_height=0.01;
    b.m_name="5";

    b.position.position[0]=0;
    b.position.position[1]=0;
    b.position.position[2]=3.0;
    b.rotation=0.0;
    boxes_.push_back(b);
    boxes_map_[QString(b.m_name.c_str())]=b;
    //    scene_3d_->addBoxEntity(b);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_l(0, 3);
    std::uniform_real_distribution<> dis_w(0, 3);
    std::uniform_real_distribution<> dis_h(0, 3);

    b.m_length=0.3;
    b.m_width=0.3;
    b.m_height=0.3;
    b.m_type="box";

    for(size_t i=0;i<25;++i){
        b.m_name=QString::number(i+6).toStdString();
        b.position.position[0]=dis_l(gen)-1.5;
        b.position.position[1]=dis_w(gen)-1.5;
        b.position.position[2]=dis_h(gen);
        b.rotation=0.0;

        boxes_.push_back(b);
        boxes_map_[QString(b.m_name.c_str())]=b;

        std::cout << b.position.position.transpose() << std::endl;

        ExtrusionStatus extr_st;
        extr_st.extr_back=true;
        extr_st.extr_front=true;
        extr_st.extr_left=true;
        extr_st.extr_right=true;
        extr_st.extr_top=true;
        extr_st.extr_bottom=true;
        extr_status_map_[QString(b.m_name.c_str())]=extr_st;
        scene_3d_->addBoxEntity(b);
    }
}


void MainWindow::on_bulletButton_clicked()
{
    testBullet();
}

void MainWindow::on_extrButton_clicked()
{
    testBullet();
}

void MainWindow::slot_reset_scene()
{
    resetScene();
}

void MainWindow::slot_update_boxes(const Boxes &bxs)
{
    //    std::cout << "Updating Boxes +++++++++++++++++++++++++++++++= " << std::endl;
    for (bpa::Box b : bxs)
    {
        scene_3d_->updateBoxEntity(b);
    }
    QMainWindow::update();
    QApplication::processEvents();

}

void MainWindow::boxSizeExtrude(float perc)
{
    //    std::cout << "Updating Boxes Extrude Sizes +++++++++++++++++++++++++++++++= " << std::endl;

    for (bpa::Box& b : boxes_)
    {
        if(b.m_type=="box"){
            ExtrusionStatus extr_st=extr_status_map_[QString(b.m_name.c_str())];

            if(extr_st.extr_top){
                double half_val=b.m_height*perc/2.0;
                half_val*=(1.5/b.position.position[2]);
                b.m_height+=half_val;
                b.position.position[2]+=half_val/2.0;
            }

            if(extr_st.extr_bottom){
                double half_val=b.m_height*perc/2.0;
                half_val*=(1.5/b.position.position[2]);
                b.m_height+=half_val;
                b.position.position[2]-=half_val/2.0;
            }

            if(extr_st.extr_front){
                double half_val=b.m_length*perc/2.0;
                half_val*=(1.5/b.position.position[2]);
                b.m_length+=half_val;
                b.position.position[0]+=half_val/2.0;
            }

            if(extr_st.extr_back){
                double half_val=b.m_length*perc/2.0;
                half_val*=(1.5/b.position.position[2]);
                b.m_length+=half_val;
                b.position.position[0]-=half_val/2.0;
            }

            if(extr_st.extr_left){
                double half_val=b.m_width*perc/2.0;
                half_val*=(1.5/b.position.position[2]);
                b.m_width+=half_val;
                b.position.position[1]+=half_val/2.0;
            }

            if(extr_st.extr_right){
                double half_val=b.m_width*perc/2.0;
                half_val*=(1.5/b.position.position[2]);
                b.m_width+=half_val;
                b.position.position[1]-=half_val/2.0;
            }

            scene_3d_->updateBoxEntity(b);
        }

        QMainWindow::update();
        QApplication::processEvents();
    }
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

void MainWindow::generateRigidBodies(std::vector<bpa::Box> &boxes)
{
    int counter=0;

    fallShapes.clear();
    fallRigidBodies.clear();
    for (bpa::Box& b : boxes)
    {
        btCollisionShape* fallShape = new btBoxShape(btVector3(b.m_length/2.0, b.m_width/2.0, b.m_height/2.0));

        btDefaultMotionState* fallMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(b.position.position[0], b.position.position[1], b.position.position[2])));
        btScalar mass = 1;
        btVector3 fallInertia(0, 0, 0);
        fallShape->calculateLocalInertia(mass, fallInertia);

        fallShapes.push_back(fallShape);
        btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
        btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
        fallRigidBody->setUserIndex(counter);
        counter++;

        fallRigidBodies.push_back(fallRigidBody);
    }
}

void MainWindow::cleanupRigidBodies()
{
    for(int n=0;n<fallRigidBodies.size();++n){
        dynamicsWorld->removeRigidBody(fallRigidBodies[n]);
        delete fallRigidBodies[n]->getMotionState();
        delete fallRigidBodies[n];
        delete fallShapes[n];
    }

}

void MainWindow::testBullet()
{
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();

    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, 0, 0));
    dynamicsWorld->setInternalTickCallback(myTickCallback, static_cast<void *>(this));

    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);

    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo
            groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    dynamicsWorld->addRigidBody(groundRigidBody);


    for (int i = 0; i < 600; i++) {

        generateRigidBodies(boxes_);
        for (int i = 0; i < fallRigidBodies.size(); i++) {
            dynamicsWorld->addRigidBody(fallRigidBodies[i]);
        }

        dynamicsWorld->stepSimulation(1 / 60.f, 10);

        for(int n=0;n<fallRigidBodies.size();++n){

            btTransform trans;
            fallRigidBodies[n]->getMotionState()->getWorldTransform(trans);

            boxes_[n].position.position[0]=trans.getOrigin().getX();
            boxes_[n].position.position[1]=trans.getOrigin().getY();
            boxes_[n].position.position[2]=trans.getOrigin().getZ();

            if(boxes_[n].m_type!="pallet_face"){
                scene_3d_->updateBoxEntity(boxes_[n]);
            }
        }

        QMainWindow::update();
        QApplication::processEvents();
        usleep(100000);
        cleanupRigidBodies();

        boxSizeExtrude(0.05);

    }

    dynamicsWorld->removeRigidBody(groundRigidBody);
    delete groundRigidBody->getMotionState();
    delete groundRigidBody;

    delete groundShape;

    delete dynamicsWorld;
    delete solver;
    delete collisionConfiguration;
    delete dispatcher;
    delete broadphase;
}

void MainWindow::testBulletExtr()
{

}

void myTickCallback(btDynamicsWorld *dynamicsWorld, btScalar timeStep)
{
    std::map<const btCollisionObject*,std::vector<btManifoldPoint*>> objectsCollisions;

    MainWindow *w = static_cast<MainWindow *>(dynamicsWorld->getWorldUserInfo());

    objectsCollisions.clear();
    int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold *contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
        auto *objA = contactManifold->getBody0();
        auto *objB = contactManifold->getBody1();
        auto& collisionsA = objectsCollisions[objA];
        auto& collisionsB = objectsCollisions[objB];
        int numContacts = contactManifold->getNumContacts();

        //        std::cout << "Contacts --> " << numContacts << std::endl;
        //        std::cout << "ObjectA --> " << objA->getUserIndex() << std::endl;
        //        std::cout << "ObjectB --> " << objB->getUserIndex() << std::endl;

        QString objA_name=QString::number(objA->getUserIndex());
        QString objB_name=QString::number(objB->getUserIndex());

        //        std::cout << "Name --> " << objA->getCollisionShape()->getName() << std::endl;

        std::vector<QVector3D > vecs;
        vecs.clear();
        for (int j = 0; j < numContacts; j++) {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            //            std::cout << "Contact " << j << std::endl;
            //            std::cout << "A --> " << pt.getPositionWorldOnA().getX() << " " << pt.getPositionWorldOnA().getY() << " " << pt.getPositionWorldOnA().getZ() << std::endl;
            //            std::cout << "B --> " << pt.getPositionWorldOnB().getX() << " " << pt.getPositionWorldOnB().getY() << " " << pt.getPositionWorldOnB().getZ() << std::endl;

            collisionsA.push_back(&pt);
            collisionsB.push_back(&pt);

            vecs.push_back(QVector3D(pt.getPositionWorldOnA().getX(), pt.getPositionWorldOnA().getY(), pt.getPositionWorldOnA().getZ()));
        }


        if(vecs.size()>2){
            QVector3D v1=vecs[1]-vecs[0];
            QVector3D v2=vecs[2]-vecs[0];
            QVector3D cp=QVector3D::crossProduct(v1,v2);

            //            qDebug() << cp;

            ExtrusionStatus extr_obj_1=w->extr_status_map_[objA_name];
            ExtrusionStatus extr_obj_2=w->extr_status_map_[objB_name];

            bpa::Box box_1=w->boxes_map_[objA_name];
            bpa::Box box_2=w->boxes_map_[objB_name];

            if(cp.x()!=0){

                if(vecs[0].x()>box_1.position.position[0]){
                    extr_obj_1.extr_front=false;
                    //                    std::cout << "Disabling Object A front extrusion" << std::endl;
                }
                else{
                    extr_obj_1.extr_back=false;
                    //                    std::cout << "Disabling Object A back extrusion" << std::endl;

                }

                if(vecs[0].x()>box_2.position.position[0]){
                    extr_obj_2.extr_front=false;
                    //                    std::cout << "Disabling Object B front extrusion" << std::endl;

                }
                else{
                    extr_obj_2.extr_back=false;
                    //                    std::cout << "Disabling Object B back extrusion" << std::endl;

                }
            }
            else if(cp.y()!=0){

                if(vecs[1].y()>box_1.position.position[1]){
                    extr_obj_1.extr_left=false;
                    //                    std::cout << "Disabling Object A left extrusion" << std::endl;

                }
                else{
                    extr_obj_1.extr_right=false;
                    //                    std::cout << "Disabling Object A right extrusion" << std::endl;

                }

                if(vecs[1].y()>box_2.position.position[1]){
                    extr_obj_2.extr_left=false;
                    //                    std::cout << "Disabling Object B left extrusion" << std::endl;

                }
                else{
                    extr_obj_2.extr_right=false;
                    //                    std::cout << "Disabling Object B right extrusion" << std::endl;
                }
            }
            else if(cp.z()!=0){
                if(vecs[2].z()>box_1.position.position[2]){
                    extr_obj_1.extr_top=false;
                    //                    std::cout << "Disabling Object A top extrusion" << std::endl;

                }
                else{
                    extr_obj_1.extr_bottom=false;
                    //                    std::cout << "Disabling Object A Bottom extrusion" << std::endl;

                }

                if(vecs[2].z()>box_2.position.position[2]){
                    extr_obj_2.extr_top=false;
                    //                    std::cout << "Disabling Object B top extrusion" << std::endl;

                }
                else{
                    extr_obj_2.extr_bottom=false;
                    //                    std::cout << "Disabling Object B bottom extrusion" << std::endl;

                }
            }
            else{

                std::cout << "Invalid Plane!!!!!!!!!!!" << std::endl;
                throw;
            }

            w->extr_status_map_[objA_name]=extr_obj_1;
            w->extr_status_map_[objB_name]=extr_obj_2;

        }
    }

    //    for( QString key: w->extr_status_map_.keys() ){
    //        qDebug() << key;// << "," << w->extr_status_map_.value( key );// << std::endl;
    //    }

    //    std::cout << "============ MAP ===================" << std::endl;
    //    foreach (ExtrusionStatus value, w->extr_status_map_){
    //        cout << value.extr_front << " " << value.extr_back << " " << value.extr_left << " " << value.extr_right << " " << value.extr_top << " " << value.extr_bottom << endl;
    //    }

    //    }
}

