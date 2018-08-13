#include "Parameters.h"

namespace bpa
{
Params::Params(const WMass& w_mass, const WVol& w_vol, const WMassVol& w_massvol, const WCom& w_com,
               const HeltRate& helt_rate, const WSupport& w_support, const WContact& w_contact,
               const NeighbourConstant& neighbour_constant, const WAssignment& w_assignment,
               const WPlaceNear& w_place_near, const BinHeight& bin_height, const MinBoxSize& min_box_size,
               const WItemInBottomArea& w_item_in_bottom_area, const WHighItemsGoodPlaced& w_high_items_good_placed,
               const GenerateSimulatedBoxes& generate_simulated_boxes,
               const StartWithAllEdgesAsFp& start_with_all_edges_as_fp, const SearchHeight& search_height,
               const SearchWidth& search_width)
  : w_mass_(w_mass.val)
  , w_vol_(w_vol.val)
  , w_massvol_(w_massvol.val)
  , w_com_(w_com.val)
  , helt_rate_(helt_rate.val)
  , w_supported_(w_support.val)
  , w_contact_(w_contact.val)
  , neighbour_constant_(neighbour_constant.val)
  , w_assignment_(w_assignment.val)
  , w_place_near_(w_place_near.val)
  , bin_height_(bin_height.val)
  , min_box_size_(min_box_size.val)
  , w_item_in_the_bottom_area_(w_item_in_bottom_area.val)
  , w_high_items_good_placed_(w_high_items_good_placed.val)
  , generate_simulated_boxes_(generate_simulated_boxes.val)
  , start_with_all_edges_as_fp_(start_with_all_edges_as_fp.val)
  , search_height_(search_height.val)
  , search_width_(search_width.val)
{
}

void Params::setAll(const WMass& w_mass, const WVol& w_vol, const WMassVol& w_massvol, const WCom& w_com,
                    const HeltRate& helt_rate, const WSupport& w_support, const WContact& w_contact,
                    const NeighbourConstant& neighbour_constant, const WAssignment& w_assignment,
                    const WPlaceNear& w_place_near, const BinHeight& bin_height, const MinBoxSize& min_box_size,
                    const WItemInBottomArea& w_item_in_bottom_area,
                    const WHighItemsGoodPlaced& w_high_items_good_placed,
                    const GenerateSimulatedBoxes& generate_simulated_boxes,
                    const StartWithAllEdgesAsFp& start_with_all_edges_as_fp, const SearchHeight& search_height,
                    const SearchWidth& search_width)
{
  w_mass_ = w_mass.val;
  w_vol_ = w_vol.val;
  w_massvol_ = w_massvol.val;
  w_com_ = w_com.val;
  helt_rate_ = helt_rate.val;
  w_supported_ = w_support.val;
  w_contact_ = w_contact.val;
  neighbour_constant_ = neighbour_constant.val;
  w_assignment_ = w_assignment.val;
  w_place_near_ = w_place_near.val;
  bin_height_ = bin_height.val;
  min_box_size_ = min_box_size.val;
  w_item_in_the_bottom_area_ = w_item_in_bottom_area.val;
  w_high_items_good_placed_ = w_high_items_good_placed.val;
  generate_simulated_boxes_ = generate_simulated_boxes.val;
  start_with_all_edges_as_fp_ = start_with_all_edges_as_fp.val;
  search_height_ = search_height.val;
  search_width_ = search_width.val;
}

double Params::w_mass() const
{
  return w_mass_;
}

double Params::w_vol() const
{
  return w_vol_;
}

double Params::w_massvol() const
{
  return w_massvol_;
}

double Params::w_com() const
{
  return w_com_;
}

double Params::helt_rate() const
{
  return helt_rate_;
}

double Params::w_supported() const
{
  return w_supported_;
}

double Params::w_contact() const
{
  return w_contact_;
}

double Params::neighbour_constant() const
{
  return neighbour_constant_;
}

double Params::w_assignment() const
{
  return w_assignment_;
}

double Params::w_place_near() const
{
  return w_place_near_;
}

double Params::bin_height() const
{
  return bin_height_;
}

double Params::min_box_size() const
{
  return min_box_size_;
}

double Params::w_item_in_the_bottom_area() const
{
  return w_item_in_the_bottom_area_;
}

double Params::w_high_items_good_placed() const
{
  return w_high_items_good_placed_;
}

bool Params::generate_simulated_boxes() const
{
  return generate_simulated_boxes_;
}

bool Params::start_with_all_edges_as_fp() const
{
  return start_with_all_edges_as_fp_;
}

int Params::search_height() const
{
  return search_height_;
}

int Params::search_width() const
{
  return search_width_;
}
}
