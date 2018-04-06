/*
 * Frames:
 *        bpp pallet frame (left corner of pallet)
 *        world pallet frame (center of pallet)
 *        base frame (world frame)
 *
 * The frames of all actors wrt base frame are in the center of actors !-!
 *
 */
#include "bppinterface.h"
#include <bpa/PhysicsBullet.h>

namespace bpainf
{

BppInterface::BppInterface()
{

    bbox_offset = 0.01;//paramJson["bbox_offset"].asDouble();
    pallet_frame_id = "p6ppallet";//paramJson["pallet_frame_id"].asString();
    pallet_length = 2.28;//paramJson["pallet_size"]["length"].asDouble();  //2.44
    pallet_width = 3.00;//paramJson["pallet_size"]["width"].asDouble();    //3.18
    pallet_height = 1.8;//paramJson["pallet_size"]["height"].asDouble();  //3.10
    pallet_center(0) = 2.34;//paramJson["pallet_center_translation"]["x"].asDouble();
    pallet_center(1) = 1.645;//paramJson["pallet_center_translation"]["y"].asDouble();
    pallet_center(2) = 0.050;//paramJson["pallet_center_translation"]["z"].asDouble();



    bpa::Params::instance()->W_MASS=0.8;
    bpa::Params::instance()->W_VOL=0.6;
    bpa::Params::instance()->W_MASSVOL=0.1;
    bpa::Params::instance()->W_COM=0.2;
    bpa::Params::instance()->HELT_RATE=0.9;
    bpa::Params::instance()->W_SUPPORTED=0.1;
    bpa::Params::instance()->W_CONTACT= 0.1;
    bpa::Params::instance()->NEIGHBOUR_CONSTANT=0.0;
    bpa::Params::instance()->W_ASSIGNMENT=0.4;
    bpa::Params::instance()->W_PLACE_NEAR=0.8;
    bpa::Params::instance()->BIN_HEIGHT=0.02;
    bpa::Params::instance()->MIN_BOX_SIZE=0.42;
    bpa::Params::instance()->W_ITEM_IN_THE_BOTTOM_AREA=0.3;
    bpa::Params::instance()->W_HIGH_ITEMS_GOOD_PLACED=0.0;
    bpa::Params::instance()->GENERATE_SIMULATED_BOXES=0;
    bpa::Params::instance()->START_WITH_ALL_EDGES_AS_FP=0;
    bpa::Params::instance()->SEARCH_HEIGHT=10;
    bpa::Params::instance()->SEARCH_WIDTH=10;

    std::cout<< "Ready..." << std::endl;
}


BppInterface::~BppInterface()
{
}


std::vector<bpa::Box> BppInterface::binPackingBoxes(std::vector<bpa::Box> &holding_area_boxes)
{


    /***************************************************************************************************
    * 2. Do bin packing
    * ************************************************************************************************/
    bpa::Bin new_pallet_config(pallet_length, pallet_width, pallet_height, bpa::Params::instance()->START_WITH_ALL_EDGES_AS_FP);

    // intial the pallet fitting points, 1 left corner fp or more
    std::vector<bpa::FittingPoint> fps = new_pallet_config.fitting_points;

    // update packed boxes and bin fitting points, state before bpp
    std::vector<bpa::Box> pallet_boxes;
    pallet_boxes.clear();
    new_pallet_config.setPackedBoxes(pallet_boxes);

    std::vector<bpa::FittingPoint> bin_fps_bpp;
    new_pallet_config.setBinFittingPoints(bin_fps_bpp);

    // build the physics world for bullet
    new_pallet_config.bulletPhysics->addBinBoundingBox();
    new_pallet_config.bulletPhysics->addNewBoxesToPhysics(new_pallet_config.packed_boxes);
    std::cout << "************************size of the  packed boxes from last bpp is :" << new_pallet_config.packed_boxes.size() <<std::endl;
    std::cout << "************************size of the fitting points from last bpp is:" << new_pallet_config.fitting_points.size() <<std::endl;

    std::cout << "\n\nBin Packing.................................................................................\n";
    bpa::BinPackingPlanner bin_packing_planner;
    new_pallet_config = bin_packing_planner.solveWithOneFunction(new_pallet_config, holding_area_boxes);

    /// get the current step packed boxes after bpp
    std::vector<bpa::Box> pack_boxes = new_pallet_config.getStepPackedBoxes();
    std::cout << "\nCurrent box plan size ==================================================" << pack_boxes.size() << std::endl;
    std::cout << "Total boxes in pallet will be = " << new_pallet_config.packed_boxes.size() << std::endl;

    for(bpa::Box b : pack_boxes)
    {
        std::cout << "Bpp UUID " << b.m_name << "  l " << b.m_length << "  w " << b.m_width << "  h " << b.m_height << "  mass " << b.m_mass <<" x " << b.position.position(0) << " y " << b.position.position(1) << " z " << b.position.position(2) << "  rotated " << b.is_rotated << "  rotation " << b.rotation << " " << b.material << "--";

        for(std::string label : b.box_labels)
        {
            std::cout <<" " << label;
        }
        std::cout << ";  " << b.is_stackable;
        std::cout <<" \n";
    }
    std::cout << "==================================================================" << std::endl;

//    /***************************************************************************************************
//    * 3. Update the bpp results to WSG
//    * ************************************************************************************************/
//    // packed boxes ---> BoxPlan, in the same frame
//    std::vector<bpp_msgs::BoxPlan> boxes_to_pack = boxesToBoxPlan(pack_boxes);
//    updateBoxPlanToWSG(boxes_to_pack);

//    std::cout << "Boxes to pack ---------------------------------> " << boxes_to_pack.size() << std::endl;

//    // boxes to actors back: Box position from inside bpp frame --> outside world base frame
//    std::vector<bpp_actor::Actor> actors_to_pack = boxesToActors(pack_boxes);
//    updateActorsToWSG(actors_to_pack);
//    std::cout << "actors_to_pack ---------------------------------> " << actors_to_pack.size() << std::endl;

//    // update the pallet fitting points
//    std::vector<bpa::FittingPoint> fitting_points = new_pallet_config.fitting_points;
//    updateBinFPsToWSG(fitting_points, false);

    return pack_boxes;
}



}


