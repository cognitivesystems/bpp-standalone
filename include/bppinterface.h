#ifndef BPP_ROS_H
#define BPP_ROS_H

#include "Parameters.h"

#include <QQuaternion>
#include <Box.h>
#include <bpp_functions.h>
#include <pp_functions.h>
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
  void setParams(const std::shared_ptr<bpa::Params>& paramsPtr);

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
  std::shared_ptr<bpa::Params> paramsPtr_;
};
}

#endif  // BPP_ROS_H
