#ifndef PARAMESTIMATOR_H
#define PARAMESTIMATOR_H

#include <opt/ParticleFilter.h>


struct BPPParams{

};

class ParamEstimator
{
public:
    ParamEstimator();

    void initialize();

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
};

#endif  // PARAMESTIMATOR_H
