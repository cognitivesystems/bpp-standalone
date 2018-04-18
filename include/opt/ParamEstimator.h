#ifndef PARAMESTIMATOR_H
#define PARAMESTIMATOR_H

#include <opt/ParticleFilter.h>
#include "bppinterface.h"
#include "Box.h"
#include <QThread>

class ParamEstimator
{
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
};

#endif  // PARAMESTIMATOR_H
