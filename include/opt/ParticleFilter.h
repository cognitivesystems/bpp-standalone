#ifndef PARTICLEFILTER_H_
#define PARTICLEFILTER_H_

#include <opt/defs.h>
#include <vector>
#include <random>
#include <boost/math/distributions/beta.hpp>
#include <iostream>
#include <random>
#include <chrono>

namespace filter{
class ParticleFilterParams{

public:
    u_int dof_;
    u_int n_particles_;
};

class Particle{
public:
    Particle(u_int dof){
        pose_.resize(dof);
        pose_p_.resize(dof);
        pose_p0_.resize(dof);
    }

    ~Particle(){
        //std::cout << "deleteing particle" << std::endl;
    }

public:
    VectorX pose_;
    VectorX pose_p_;
    VectorX pose_p0_;

    float w;          /**< weight */
};

class ParticleFilter{
public:

    ParticleFilter(uint id);
    ~ParticleFilter();

    void initialize(const VectorX& alpha, const VectorX& beta, size_t n_particles=100);

    void predict();

    void predict_with_simple_model();

    void predict_with_beta_distribution();

    void correct();

    void resizeParticleSet(const size_t new_count);

    void setParticleWeight(u_int id, float w);

    size_t nOfParticles(){
        return mParticles.size();
    }

    static int particle_cmp(const void* p1,const void* p2 ){
        Particle* _p1 = (Particle*)p1;
        Particle* _p2 = (Particle*)p2;

        if( _p1->w > _p2->w )
            return -1;
        if( _p1->w < _p2->w )
            return 1;
        return 0;
    }

    void getParticlePose(const int id, VectorX& pose);

    VectorX getOutputPose();

    double getAverageWeight();

    double getMaxLikelihood();

    Particle* getMaxParticle();

    uint getFilterId(){
        return filter_id_;
    }

    float getBetaD(size_t filter_id){
        float rand=dis(generator);
        return quantile(distributions[filter_id], rand);
    }

    std::pair<double,double> getMeanVariance(const std::vector<float >& vec) {
        float mean = 0, M2 = 0, variance = 0;

        size_t n = vec.size();
        for(size_t i = 0; i < n; ++i) {
            float delta = vec[i] - mean;
            mean += delta / (i + 1);
            M2 += delta * (vec[i] - mean);
            variance = M2 / (i + 1);
            if (i >= 2) {
                // <-- You can use the running mean and variance here
            }
        }

        return std::make_pair(mean, variance);
    }

    static double argInit_real_T(void)
    {
        return 0.0;
    }


    VectorX getAlpha(){
        return alpha_;
    }

    VectorX getBeta(){
        return beta_;
    }

    VectorX getCovariance();

private:
    void normalize();

    void updateDistribution();

protected:
    uint filter_id_;

    ParticleFilterParams mParams;

    std::vector<Particle > mParticles;

    VectorX mOutputPose;
    
    std::random_device rd;
    std::mt19937 generator;
    std::uniform_real_distribution<> dis;
    std::vector<  boost::math::beta_distribution< > > distributions;

    VectorX alpha_;
    VectorX beta_;
    double step=0.1;
};


}
#endif /* PARTICLEFILTER_H_ */
