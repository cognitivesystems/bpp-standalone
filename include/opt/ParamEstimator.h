#ifndef PARAMESTIMATOR_H
#define PARAMESTIMATOR_H

#include <opt/ParticleFilter.h>
#include "bppinterface.h"
#include "Box.h"
#include <QApplication>
#include <QMainWindow>
#include <QObject>
#include <QtDataVisualization>
#include <QtCharts>
#include <QChart>
#include <QLineSeries>
#include <opt/pdflib.hpp>

using namespace QtDataVisualization;

class ParamEstimator : public QObject
{
    Q_OBJECT

public:
    ParamEstimator();

    void initialize();

    void setBoxData(std::vector<bpa::Box> boxes);

    std::vector<bpa::Box> stepNext();

    void run();

private:
    double eval_bpp(const VectorX data);

    double compute_used_space(const std::vector<bpa::Box>& boxes)
    {
        double bin_volume = 0.0;
        for(bpa::Box b : boxes)
        {
            bin_volume += b.getVolume();
        }
        return bin_volume;
    }

    double compute_total_mass(const std::vector<bpa::Box>& boxes)
    {
        double bin_mass = 0.0;
        for(bpa::Box b : boxes)
        {
            bin_mass += b.m_mass;
        }
        return bin_mass;
    }

    Eigen::Vector3d compute_bin_com(const std::vector<bpa::Box>& boxes)
    {
        Eigen::Vector3d bin_com, box_com;
        //        double bin_mass = 0.0;
        //        for(actor_msgs::Actor b : actors)
        //        {
        //            box_com << b.desiredPoseVec[0].position.x, b.desiredPoseVec[0].position.y, b.desiredPoseVec[0].position.z;
        //            bin_com = (box_com * b.weight + bin_com * bin_mass) / (bin_mass + b.weight);
        //            bin_mass += b.weight;
        //        }
        return bin_com;
    }

    std::vector<double> linspace(double a, double b, int n) {
        std::vector<double> array;
        double step = (b-a) / (n-1);

        while(a <= b) {
            array.push_back(a);
            a += step;           // could recode to better handle rounding errors
        }
        return array;
    }

    std::vector<double > computePDF(const double alpha, const double beta){

        std::vector<double > y_est;
        y_est=x_target;
        for(size_t i=0;i<y_est.size();++i){
            y_est[i]=r8_beta_pdf(alpha, beta, x_target[i]);
        }

        return y_est;
    }

private:
    filter::ParticleFilter pf_;
    VectorX alpha_;
    VectorX beta_;

    std::vector<bool > dim_active_;
    size_t dims_=0;
    size_t n_particles_;

    VectorX pose;

    std::vector<bpa::Box> est_boxes_;
    std::vector<bpa::Box> est_planned_boxes_;

    std::random_device rd;
    std::mt19937 generator;
    std::uniform_real_distribution<> dis;


    //visualisation
    std::vector<double > x_target;
    std::vector<double > y_target;

    QLineSeries *series0;
    QLineSeries *series1;
    QLineSeries *series2;
    QLineSeries *series3;
    QLineSeries *series4;

    QChart *chart;

    QChartView *chartView;


    std::vector<double > pdf0;
    std::vector<double > pdf1;
    std::vector<double > pdf2;
    std::vector<double > pdf3;
    std::vector<double > pdf4;

    QMainWindow window;

};

#endif  // PARAMESTIMATOR_H
