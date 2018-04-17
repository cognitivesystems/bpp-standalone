#include <opt/ParamEstimator.h>

ParamEstimator::ParamEstimator()
    :pf_(0), n_particles_(1000), generator(rd()), dis(0.0, 1.0)
{

}

void ParamEstimator::initialize()
{
    dims_=3;
    alpha_.resize(dims_);
    beta_.resize(dims_);
    for(size_t n=0;n<dims_;++n){
        alpha_[n]=1000.0;
        beta_[n]=1000.0;
    }

    for(size_t n=0;n<dim_active_.size();++n){
        if(n<5){
            dim_active_[n]=true;
            dims_++;
        }
        else
            dim_active_[n]=false;
    }

    if(n_particles_%4!=0){
        std::cout << "ERROR: n_particles not a multiple of 4" << std::endl;
        throw;
    }

    pf_.initialize(alpha_, beta_, n_particles_);

    pose.resize(dims_);
}

void ParamEstimator::setBoxData(std::vector<bpa::Box> boxes){

    est_boxes_=boxes;
}

std::vector<bpa::Box> ParamEstimator::stepNext()
{

}

void ParamEstimator::run()
{
    while(true){

        pf_.predict();
        for(size_t n=0;n<pf_.nOfParticles();n+=4){

#pragma omp parallel for
            for(size_t t=0;t<4;++t){
                int p_idx=n+t;
                std::cout << "particle --> " << p_idx << std::endl;

                double likelihood=0.0;
                pf_.getParticlePose(p_idx, pose);
                std::cout << "pose --> " ;
                for(size_t p=0;p<pose.size();++p){
                    std::cout << pose[p] << " ";
                }
                std::cout << "\n";

                likelihood=eval_bpp(pose);

                std::cout << "likelihood --> " << likelihood << std::endl;
                pf_.setParticleWeight(p_idx, likelihood);
            }
        }

        pf_.correct();
    }
}

double ParamEstimator::eval_bpp(const VectorX data)
{
    return dis(generator);

}
