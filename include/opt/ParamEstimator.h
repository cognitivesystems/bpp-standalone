#ifndef PARAMESTIMATOR_H
#define PARAMESTIMATOR_H

#include <opt/ParticleFilter.h>
#include <bpa/Box.h>

struct BPPParams{

};

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

};

#endif  // PARAMESTIMATOR_H
