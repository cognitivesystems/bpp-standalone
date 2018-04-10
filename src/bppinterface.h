#ifndef BPP_ROS_H
#define BPP_ROS_H

#include <QQuaternion>
#include <bpa/Box.h>
#include <bpa/bpp_functions.h>
#include <bpa/pp_functions.h>
#include <iomanip>
#include <iostream>

namespace bpainf
{
/**
     * \class   BppROS
     */
class BppInterface
{
public:
  // \brief  Constructor
  BppInterface();

  // \brief  Destructor
  ~BppInterface();

  std::vector<bpa::Box> binPackingBoxes(std::vector<bpa::Box>& holding_area_boxes);

  void loadParamsJSON(std::string& file_name);

private:
  /** *** member functions *****/
  // \brief  Function to palletise boxes.

  double pallet_length;
  double pallet_width;
  double pallet_height;
  Eigen::Vector3d pallet_center;
  std::string pallet_frame_id;
  double bbox_offset;

  std::string parameter_file;
};
}

#endif  // BPP_ROS_H
