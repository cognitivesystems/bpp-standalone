#include <QJsonArray>
#include <QFile>
#include <QDebug>
#include <QJsonDocument>
#include <QtCore/QDir>

#include "BoxJsonParser.h"

namespace bpa
{
double BoxJsonParser::bbox_offset_ = 0.01;

std::vector<bpa::Box> BoxJsonParser::getBoxesFromJsonFile(const QString& fileName)
{
  std::vector<bpa::Box> boxes;

  QFile file(fileName);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    std::cerr << "Failed to open json file: " << fileName.toStdString() << std::endl;
  }

  QJsonParseError err;
  QJsonDocument doc(QJsonDocument::fromJson(file.readAll(), &err));
  file.close();

  if (doc.isNull())
  {
    std::cerr << "Json file parse error: " << err.errorString().toStdString() << std::endl;
  }

  for (const QJsonValueRef& v : doc.array())
  {
    bpa::Box box = getBoxFromJsonObject(v.toObject());
    boxes.push_back(box);
  }

  return boxes;
}

bpa::Box BoxJsonParser::getBoxFromJsonObject(const QJsonObject& jsonObject)
{
  bpa::Box box;
  box.m_length = jsonObject["bbox"].toObject()["x"].toDouble();  // + bbox_offset_;
  box.m_width = jsonObject["bbox"].toObject()["y"].toDouble();   // + bbox_offset_;
  box.m_height = jsonObject["bbox"].toObject()["z"].toDouble();
  box.m_mass = jsonObject["weight"].toDouble();
  box.m_name = jsonObject["uuid"].toString().toStdString();
  box.material = jsonObject["material"].toString().toStdString();

  box.tool_name = jsonObject["robotTool"].toString().toStdString();

  for (const QJsonValueRef& label : jsonObject["labels"].toArray())
  {
    box.box_labels.push_back(label.toString().toStdString());
  }

  std::string state = jsonObject["state"].toString().toStdString();
  if (state == "packed")
  {
    box.is_packed = true;
  }

  std::string type = jsonObject["type"].toString().toStdString();
  box.m_type = type;

  if (type == "other" || type == "pallet")
  {
    box.is_stackable = false;
  }

  QJsonValue value = jsonObject.value("targetPoseVec");
  QJsonArray array = value.toArray();

  foreach (const QJsonValue& v, array)
  {
    box.position << v.toObject()["position"].toObject()["x"].toDouble(),
        v.toObject()["position"].toObject()["y"].toDouble(), v.toObject()["position"].toObject()["z"].toDouble();

    break;
  }

  box.is_rotated = false;
  box.rotation = 0.0;

  return box;
}
}
