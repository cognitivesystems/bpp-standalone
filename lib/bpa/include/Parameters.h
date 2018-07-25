#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <iostream>
#include <cmath>
#include <limits>

namespace bpa
{
class Params
{
public:
  Params() = default;
  Params(double w_mass, double w_vol, double w_massvol, double w_com, double helt_rate, double w_supported,
         double w_contact, double neighbour_constant, double w_assignment, double w_place_near, double bin_height,
         double min_box_size, double w_item_in_the_bottom_area, double w_high_items_good_placed,
         bool generate_simulated_boxes, bool start_with_all_edges_as_fp, int search_height, int search_width);

  Params(const Params& rhs) = default;
  Params& operator=(const Params& rhs) = default;

  Params(Params&& rhs) = default;
  Params& operator=(Params&& rhs) = default;

  void setAll(double w_mass, double w_vol, double w_massvol, double w_com, double helt_rate, double w_supported,
              double w_contact, double neighbour_constant, double w_assignment, double w_place_near, double bin_height,
              double min_box_size, double w_item_in_the_bottom_area, double w_high_items_good_placed,
              bool generate_simulated_boxes, bool start_with_all_edges_as_fp, int search_height, int search_width);

  double w_mass() const;
  double w_vol() const;
  double w_massvol() const;

  double w_com() const;
  double helt_rate() const;
  double w_supported() const;
  double w_contact() const;

  double neighbour_constant() const;
  double w_assignment() const;
  double w_place_near() const;
  double bin_height() const;
  double min_box_size() const;

  double w_item_in_the_bottom_area() const;
  double w_high_items_good_placed() const;

  bool generate_simulated_boxes() const;
  bool start_with_all_edges_as_fp() const;

  int search_height() const;
  int search_width() const;

private:
  /*< --------------------------------------------- */
  /*< Weighting Parameters of the Scoring Functions */
  double w_mass_;
  double w_vol_;
  double w_massvol_;

  double w_com_;
  double helt_rate_;   /* Minimum Percentage of ground surface of a Box which has to be supported by other boxes or the
                       pallet underneath */
  double w_supported_; /* The box support area */
  double w_contact_;   /* The box contact areas with surounding */

  double neighbour_constant_;
  double w_assignment_;
  double w_place_near_;
  double bin_height_;   /* The box reaches the bin max height. */
  double min_box_size_; /* The min box size(width). */

  double w_item_in_the_bottom_area_;
  double w_high_items_good_placed_;
  /*< --------------------------------------------- */

  bool generate_simulated_boxes_;   /* Indicating if Simulated Boxes should be generated*/
  bool start_with_all_edges_as_fp_; /* Indicating if all four edges should be used asinitial Fitting Points*/

  int search_height_; /* Indicating the Search height of the Deep Search Algorithms*/
  int search_width_;  /* Indicating the Search width of the Deep Search Algorithms*/
};

// const PARAMETERS params =
//{
//    0.3,                  // W_MASS:   0.3
//    0.6,                  // W_VOL : 0.6 / 0.8
//    0.1,                  // W_MASSVOL: 0.3 / 0.1

//    0.2,                  // W_COM      0.0/0.2
//    1.0,                  // HELT_RATE: 0.7 / 0.9
//    0.1,                  // W_SUPPORTED
//    0.1,                  // W_CONTACT   (with this look better!)

//    0.0,                  // NEIGHBOUR_CONSTANT  0.1 (8.png)/0.0(5.png):  0.1, looks better for new data
//    0.3,                  // W_ASSIGNMENT        0.4/0.3
//    0.8,                  // W_PLACE_NEAR        0.6/0.0/0.8 ??
//    0.02,                 // If the top box is reach the max height
//    0.3,                  // The min box size(width).

//    0.3,                  // W_ITEM_IN_THE_BOTTOM_AREA /0.3
//    0.3,                  // W_HIGH_ITEMS_GOOD_PLACED

//    false,                // GENERATE_SIMULATED_BOXES
//    false,                // START_WITH_ALL_EDGES_AS_FP
//    10,                   // SEARCH_HEIGHT
//    10                   // SEARCH_WIDTH
//};

struct supportingBox
{
  std::string uuid;
  double helt;
};
}

//#define FLOAT_EPS std::numeric_limits<float>::epsilon()
#define FLOAT_EPS 0.00001

inline bool floatEqual(double a, double b)
{
  return std::fabs(a - b) < FLOAT_EPS;
}

inline bool floatLessThan(double a, double b)
{
  return ((std::fabs(a - b) > FLOAT_EPS) && ((a - b) < FLOAT_EPS));
}

inline bool floatGreaterThan(double a, double b)
{
  return ((std::fabs(a - b) > FLOAT_EPS) && ((a - b) > FLOAT_EPS));
}

inline bool floatGreaterEqual(double a, double b)
{
  return (floatEqual(a, b) || floatGreaterThan(a, b));
}

inline bool floatLessEqual(double a, double b)
{
  return (floatEqual(a, b) || floatLessThan(a, b));
}

#endif
