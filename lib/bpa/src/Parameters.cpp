#include "Parameters.h"

namespace bpa
{
Params::Params(double w_mass, double w_vol, double w_massvol, double w_com, double helt_rate, double w_supported,
               double w_contact, double neighbour_constant, double w_assignment, double w_place_near, double bin_height,
               double min_box_size, double w_item_in_the_bottom_area, double w_high_items_good_placed,
               bool generate_simulated_boxes, bool start_with_all_edges_as_fp, int search_height, int search_width)
  : w_mass_(w_mass)
  , w_vol_(w_vol)
  , w_massvol_(w_massvol)
  , w_com_(w_com)
  , helt_rate_(helt_rate)
  , w_supported_(w_supported)
  , w_contact_(w_contact)
  , neighbour_constant_(neighbour_constant)
  , w_assignment_(w_assignment)
  , w_place_near_(w_place_near)
  , bin_height_(bin_height)
  , min_box_size_(min_box_size)
  , w_item_in_the_bottom_area_(w_item_in_the_bottom_area)
  , w_high_items_good_placed_(w_high_items_good_placed)
  , generate_simulated_boxes_(generate_simulated_boxes)
  , start_with_all_edges_as_fp_(start_with_all_edges_as_fp)
  , search_height_(search_height)
  , search_width_(search_width)
{
}

void Params::setAll(double w_mass, double w_vol, double w_massvol, double w_com, double helt_rate, double w_supported,
                    double w_contact, double neighbour_constant, double w_assignment, double w_place_near,
                    double bin_height, double min_box_size, double w_item_in_the_bottom_area,
                    double w_high_items_good_placed, bool generate_simulated_boxes, bool start_with_all_edges_as_fp,
                    int search_height, int search_width)
{
  w_mass_ = w_mass;
  w_vol_ = w_vol;
  w_massvol_ = w_massvol;
  w_com_ = w_com;
  helt_rate_ = helt_rate;
  w_supported_ = w_supported;
  w_contact_ = w_contact;
  neighbour_constant_ = neighbour_constant;
  w_assignment_ = w_assignment;
  w_place_near_ = w_place_near;
  bin_height_ = bin_height;
  min_box_size_ = min_box_size;
  w_item_in_the_bottom_area_ = w_item_in_the_bottom_area;
  w_high_items_good_placed_ = w_high_items_good_placed;
  generate_simulated_boxes_ = generate_simulated_boxes;
  start_with_all_edges_as_fp_ = start_with_all_edges_as_fp;
  search_height_ = search_height;
  search_width_ = search_width;
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
