#ifndef PARAMS_TYPES_H
#define PARAMS_TYPES_H

namespace bpa
{
struct WMass
{
  explicit WMass(double d) : val(d)
  {
  }
  double val;
};

struct WVol
{
  explicit WVol(double d) : val(d)
  {
  }
  double val;
};

struct WMassVol
{
  explicit WMassVol(double d) : val(d)
  {
  }
  double val;
};

struct WCom
{
  explicit WCom(double d) : val(d)
  {
  }
  double val;
};

struct HeltRate
{
  explicit HeltRate(double d) : val(d)
  {
  }
  double val;
};

struct WSupport
{
  explicit WSupport(double d) : val(d)
  {
  }
  double val;
};

struct WContact
{
  explicit WContact(double d) : val(d)
  {
  }
  double val;
};

struct NeighbourConstant
{
  explicit NeighbourConstant(double d) : val(d)
  {
  }
  double val;
};

struct WAssignment
{
  explicit WAssignment(double d) : val(d)
  {
  }
  double val;
};

struct WPlaceNear
{
  explicit WPlaceNear(double d) : val(d)
  {
  }
  double val;
};

struct BinHeight
{
  explicit BinHeight(double d) : val(d)
  {
  }
  double val;
};

struct MinBoxSize
{
  explicit MinBoxSize(double d) : val(d)
  {
  }
  double val;
};

struct WItemInBottomArea
{
  explicit WItemInBottomArea(double d) : val(d)
  {
  }
  double val;
};

struct WHighItemsGoodPlaced
{
  explicit WHighItemsGoodPlaced(double d) : val(d)
  {
  }
  double val;
};

struct GenerateSimulatedBoxes
{
  explicit GenerateSimulatedBoxes(bool b) : val(b)
  {
  }
  bool val;
};

struct StartWithAllEdgesAsFp
{
  explicit StartWithAllEdgesAsFp(bool b) : val(b)
  {
  }
  bool val;
};

struct SearchHeight
{
  explicit SearchHeight(int i) : val(i)
  {
  }
  int val;
};

struct SearchWidth
{
  explicit SearchWidth(int i) : val(i)
  {
  }
  int val;
};
}  // namespace bpa

#endif  // PARAMS_TYPES_H