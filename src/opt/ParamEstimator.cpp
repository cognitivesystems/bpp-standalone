#include <opt/ParamEstimator.h>
#include "mainwindow.h"

ParamEstimator::ParamEstimator()
    :pf_(0), n_particles_(20), generator(rd()), dis(0.0, 1.0)
{

        qRegisterMetaType<std::vector<bpa::Box> >("Boxes");

}

void ParamEstimator::initialize()
{
    dims_=5;
    alpha_.resize(dims_);
    beta_.resize(dims_);
    for(size_t n=0;n<dims_;++n){
        alpha_[n]=100.0;
        beta_[n]=100.0;
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
    //while(true){

        pf_.predict();
        for(size_t n=0;n<pf_.nOfParticles();n+=4){

#pragma omp parallel for
            for(size_t t=0;t<4;++t){
                int p_idx=n+t;

                double likelihood=0.0;
                pf_.getParticlePose(p_idx, pose);
                std::cout << "pose --> " ;
                for(size_t p=0;p<pose.size();++p){
                    std::cout << pose[p] << " ";
                }
                std::cout << "\n";

                likelihood=eval_bpp(pose);

//                std::cout << "likelihood --> " << likelihood << std::endl;
                std::cout << "particle --> " << p_idx << "  likelihood ---> " << likelihood << std::endl;

                pf_.setParticleWeight(p_idx, likelihood);

            }
        }

        pf_.correct();

        std::cout << "Average Weight ----> " << pf_.getAverageWeight() << std::endl;

    //}
}

double ParamEstimator::eval_bpp(const VectorX data)
{
    bpainf::BppInterface bpp;
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
    bool start_with_all_edges_as_fp=true;
    int search_height=10;
    int search_width=10;

    paramsPtr.get()->setAll(data[0], data[1], data[2], data[3], helt_rate, w_supported, data[4],
            neightbour_constant, w_assignment, w_place_near, bin_height, min_box_size, w_item_in_the_bottom_area, w_high_items_good_placed,
            generate_simulated_boxes, start_with_all_edges_as_fp, search_height, search_width);

    bpp.setParams(paramsPtr);

    std::vector<bpa::Box> boxes=est_boxes_;

    for (bpa::Box& b : boxes)
    {
        b.position[0] -= b.m_length / 2;
        b.position[1] -= b.m_width / 2;
        b.position[2] -= b.m_height / 2;
    }

    est_planned_boxes_ = bpp.binPackingBoxes(boxes);

    static double norm_val = 2.2*3.0*3.0;

    double used_space=compute_used_space(est_planned_boxes_);

    for (bpa::Box& b : est_planned_boxes_)
    {
        b.position[0] += b.m_length / 2;
        b.position[1] += b.m_width / 2;
        b.position[2] += b.m_height / 2;
    }

    send_reset_scene();
    send_update_boxes(est_planned_boxes_);

    return std::max(0.0, used_space/norm_val);
}
