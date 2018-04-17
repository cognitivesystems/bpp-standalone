#include "opt/ParticleFilter.h"

namespace filter
{

ParticleFilter::ParticleFilter(uint id)
    : filter_id_(id),generator(rd()), dis(0.0, 1.0)
{


}

ParticleFilter::~ParticleFilter()
{
    mParticles.erase(mParticles.begin(), mParticles.end());
    mParticles.clear();

}

void ParticleFilter::initialize(const VectorX &alpha, const VectorX &beta, size_t n_particles)
{

    mParams.dof_=alpha.size();
    mParams.n_particles_=n_particles;

    alpha_=alpha;
    beta_=beta;

    std::cout << "PARTICLE_FILTER " << std::endl;
    std::cout << "dof_ " << mParams.dof_ << std::endl;
    std::cout << "n_particles_ " << mParams.n_particles_ << std::endl;

    distributions.resize(mParams.dof_);

    VectorX cov;
    cov.resize(mParams.dof_);

    for(size_t n=0;n<mParams.dof_;++n){
        boost::math::beta_distribution<> beta_d(0.5, 0.5);
        distributions[n]=beta_d;
        cov[n]=200.0;
    }

    cov[2]=0.0;

    for(size_t n=0;n<mParams.n_particles_;++n){
        mParticles.push_back(Particle(mParams.dof_));
    }

    if(alpha.size()!=mParams.dof_){
        std::cout << "Init Pose does not match dof " << mParams.dof_ << std::endl;
        throw;
    }

    /* create particles at the centers of each of n regions */

    //std::cout << "initializing particles at " << initPose[0] << " " << initPose[1] << " " << initPose[2] << std::endl;
    for(size_t i=0; i<mParams.n_particles_ ; ++i){

        for(size_t n=0;n<mParams.dof_;++n){
            mParticles[i].pose_p0_[n] = mParticles[i].pose_p_[n] = mParticles[i].pose_[n] = getBetaD(n);
            mParticles[i].w  = 0.0f;
        }
    }
}


void ParticleFilter::resizeParticleSet(const size_t new_count)
{
    std::vector<Particle > mParticles_old=mParticles;

    mParticles.erase(mParticles.begin(), mParticles.end());
    mParticles.clear();
    for(size_t n=0;n<new_count;++n)
        mParticles.push_back(Particle(mParams.dof_));

    for(size_t i=0; i<mParticles.size(); ++i)
        mParticles[i]=mParticles_old[i];

    if(mParticles.size()>mParticles_old.size() && mParticles_old.size()>0){
        for(size_t i=mParticles_old.size(); i<mParticles.size(); ++i)
            mParticles[i]=mParticles_old[0];
    }

    mParams.n_particles_=mParticles.size();

}

void ParticleFilter::predict()
{
    predict_with_beta_distribution();

}

void ParticleFilter::predict_with_simple_model()
{
//    static Particle p(mParams.dof_);
//    p.pose_.resize(mParams.dof_);
//    p.pose_p_.resize(mParams.dof_);
//    p.pose_p0_.resize(mParams.dof_);

//    for(size_t i=0; i<mParams.n_particles_; ++i){
//        for(size_t n=0;n<mParams.dof_;++n){
//            p.pose_[n] = mParticles[i].pose_p_[n]+distributions[n](generator);
//            p.pose_p_[n] = mParticles[i].pose_[n];
//            p.pose_p0_[n] = mParticles[i].pose_p0_[n];
//            p.w = 0.0;

//            mParticles[i].pose_[n]  = p.pose_[n];
//            mParticles[i].pose_p_[n]  = p.pose_p_[n] ;
//            mParticles[i].pose_p0_[n]  = p.pose_p0_[n] ;
//            mParticles[i].w = p.w;
//        }
//    }
}

void ParticleFilter::correct()
{
    normalize();

    updateDistribution();
}

void ParticleFilter::normalize()
{
    float sum = 0.0f;

    for(size_t i=0; i<mParams.n_particles_; ++i)
        sum += mParticles[i].w;

    if(sum > 0){
        for(size_t i=0; i<mParams.n_particles_; ++i )
            mParticles[i].w /= sum;
    }
    else{
        for(size_t i=0; i<mParams.n_particles_; ++i )
            mParticles[i].w = 0.0f;
    }
}


void ParticleFilter::updateDistribution()
{
    std::qsort( mParticles.data(), mParticles.size(), sizeof( Particle ), particle_cmp );

    int length=(int)(mParticles.size()*0.1);
    std::cout << "length ----------------> " << length << std::endl;

    for(size_t n=0;n<mParams.dof_;++n){
        std::vector<float > datavec;
        datavec.resize(length);

        for(size_t i=0;i<length;++i){
            Particle p = mParticles[i];
            datavec[i]=p.pose_[n];
        }

        std::cout << __LINE__ << std::endl;


        std::pair <float, float> mv = getMeanVariance(datavec);

        float dist_params[2];
        dist_params[0]=distributions[n].find_alpha(mv.first, mv.second);
        dist_params[1]=distributions[n].find_beta(mv.first, mv.second);

        alpha_[n]=alpha_[n]*step+(1.0-step)*dist_params[0];
        beta_[n]=beta_[n]*step+(1.0-step)*dist_params[1];
    }

    distributions.erase(distributions.begin(), distributions.end());
    distributions.clear();
    distributions.resize(mParams.dof_);

    std::cout << "++++++++++ Updating Distribution Params +++++++++++++++" << std::endl;
    for(size_t n=0;n<mParams.dof_;++n){

        boost::math::beta_distribution<> beta_d(alpha_[n], beta_[n]);
        distributions[n]=beta_d;
        std::cout << alpha_[n] << " " << beta_[n] << std::endl;
    }
}

void ParticleFilter::setParticleWeight(u_int id, float w)
{
    mParticles[id].w = w;
}

VectorX ParticleFilter::getOutputPose()
{
    return getMaxParticle()->pose_;
}

double ParticleFilter::getAverageWeight()
{
    double w_sum=0.0;
    for(size_t i=0; i<mParams.n_particles_; ++i)
        w_sum+=mParticles[i].w;

    w_sum/=mParams.n_particles_;

    return w_sum;

}

double ParticleFilter::getMaxLikelihood()
{
    double w_max=0.0;
    for(size_t i=0; i<mParams.n_particles_; ++i){
        if(w_max<mParticles[i].w)
            w_max=mParticles[i].w;
    }

    return w_max;
}

Particle *ParticleFilter::getMaxParticle()
{
    double w_max=0.0;
    int id_max=0.0;
    for(size_t i=0; i<mParams.n_particles_; ++i){
        if(w_max<mParticles[i].w){
            w_max=mParticles[i].w;
            id_max=i;
        }
    }

    return &mParticles[id_max];
}

VectorX ParticleFilter::getCovariance()
{
    VectorX covar;
    covar.resize(mParams.dof_);
    for(size_t n=0;n<mParams.dof_;++n){
        double var=0.0;
        double mean=0.0;
        for(size_t i=0; i<mParams.n_particles_; ++i){
            Particle p = mParticles[i];
            mean+=p.pose_[n];
        }
        mean/=(double)(mParams.n_particles_);

        for(size_t i=0; i<mParams.n_particles_; ++i){
            Particle p = mParticles[i];
            var += std::pow(p.pose_[n] - mean,2);
        }
        var/=(double)(mParams.n_particles_);
        covar[n]=var;
    }

    return covar;
}

void ParticleFilter::getParticlePose(const int id, VectorX &pose)
{
    pose=mParticles[id].pose_;
}

void ParticleFilter::ParticleFilter::predict_with_beta_distribution()
{
    static Particle p(mParams.dof_);
    p.pose_.resize(mParams.dof_);
    p.pose_p_.resize(mParams.dof_);
    p.pose_p0_.resize(mParams.dof_);

    for(size_t i=0; i<mParams.n_particles_; ++i){
//                std::cout << "particle " ;
        for(size_t n=0;n<mParams.dof_;++n){
            p.pose_[n] = getBetaD(n);
            p.pose_p_[n] = mParticles[i].pose_[n];
            p.pose_p0_[n] = mParticles[i].pose_p0_[n];
            p.w = 0.0;

            mParticles[i].pose_[n]  = p.pose_[n];
            mParticles[i].pose_p_[n]  = p.pose_p_[n] ;
            mParticles[i].pose_p0_[n]  = p.pose_p0_[n] ;
            mParticles[i].w = p.w;

//                        std::cout << i << " " << mParticles[i].pose_[n] << " ";
        }
//                std::cout << "\n";
    }
}

}

