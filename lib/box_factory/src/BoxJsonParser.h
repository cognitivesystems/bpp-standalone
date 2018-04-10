#ifndef BOX_JSON_PARSER_H
#define BOX_JSON_PARSER_H

#include <string>
#include <vector>
#include <QJsonObject>

#include "Box.h"

namespace box_factory
{
class BoxJsonParser
{
public:
  BoxJsonParser() = default;
  ~BoxJsonParser() = default;

  static std::vector<bpa::Box> getBoxesFromJsonFile(const QString& jsonFile);
  static bpa::Box getBoxFromJsonObject(const QJsonObject& jsonObject);

private:
  static double bbox_offset_;
};
}

#endif  // BOX_JSON_PARSER_H