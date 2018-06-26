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
#include "scenerenderer3d.h"
#include "PhysicsBullet.h"

namespace bpainf
{
BppInterface::BppInterface()
  : pallet_length(2.44)
  , pallet_width(3.18)
  , pallet_height(3.10)
  , bbox_offset(0.01)
  , paramsPtr_(std::make_shared<bpa::Params>(0.8, 0.6, 0.1, 0.2, 0.9, 0.1, 0.1, 0.0, 0.4, 0.8, 0.02, 0.42, 0.3, 0.0, 0,
                                             0, 10, 10))
{
  // paramJson["bbox_offset"].asDouble();
  pallet_frame_id = "p6ppallet";  // paramJson["pallet_frame_id"].asString();
                                  // paramJson["pallet_size"]["length"].asDouble(); //2.44
                                  // paramJson["pallet_size"]["width"].asDouble(); //3.18
                                  // paramJson["pallet_size"]["height"].asDouble();  //3.10
  pallet_center(0) = 2.34;        // paramJson["pallet_center_translation"]["x"].asDouble();
  pallet_center(1) = 1.645;       // paramJson["pallet_center_translation"]["y"].asDouble();
  pallet_center(2) = 0.050;       // paramJson["pallet_center_translation"]["z"].asDouble();

//  std::cout << "Ready..." << std::endl;
}

BppInterface::~BppInterface()
{
}

std::vector<bpa::Box> BppInterface::binPackingBoxes(std::vector<bpa::Box>& holding_area_boxes)
{
  for (bpa::Box& box : holding_area_boxes)
  {
    box.setParams(paramsPtr_);
  }

  /***************************************************************************************************
  * 2. Do bin packing
  * ************************************************************************************************/
  bpa::Bin new_pallet_config(pallet_length, pallet_width, pallet_height, paramsPtr_->start_with_all_edges_as_fp());
  new_pallet_config.setParams(paramsPtr_);

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
//  std::cout << "************************size of the  packed boxes from last bpp is :"
//            << new_pallet_config.packed_boxes.size() << std::endl;
//  std::cout << "************************size of the fitting points from last bpp is:"
//            << new_pallet_config.fitting_points.size() << std::endl;

//  std::cout << "\n\nBin "
//               "Packing........................................................"
//               ".........................\n";
  bpa::BinPackingPlanner bin_packing_planner;
  bin_packing_planner.setParams(paramsPtr_);
  new_pallet_config = bin_packing_planner.solveWithOneFunction(new_pallet_config, holding_area_boxes);

  /// get the current step packed boxes after bpp
  std::vector<bpa::Box> pack_boxes = new_pallet_config.getStepPackedBoxes();

//  std::cout << "\nCurrent box plan size "
//               "=================================================="
//            << pack_boxes.size() << std::endl;
//  std::cout << "Total boxes in pallet will be = " << new_pallet_config.packed_boxes.size() << std::endl;

//  for (bpa::Box b : pack_boxes)
//  {
//    std::cout << "Bpp UUID " << b.m_name << "  l " << b.m_length << "  w " << b.m_width << "  h " << b.m_height
//              << "  mass " << b.m_mass << " x " << b.position.position(0) << " y " << b.position.position(1) << " z "
//              << b.position.position(2) << "  rotated " << b.is_rotated << "  rotation " << b.rotation << " "
//              << b.material << "--";

//    for (std::string label : b.box_labels)
//    {
//      std::cout << " " << label;
//    }
//    std::cout << ";  " << b.is_stackable;
//    std::cout << " \n";
//  }
//  std::cout << "==================================================================" << std::endl;
  return pack_boxes;
}

void BppInterface::setParams(const std::shared_ptr<bpa::Params>& paramsPtr)
{
  paramsPtr_ = paramsPtr;
}
}
