#include <gtest/gtest.h>
#include <QJsonObject>
#include <QDir>
#include <QJsonDocument>
#include "BoxJsonParser.h"

namespace box_factory
{
class BoxJsonParserTestFixture : public testing::Test
{
protected:
  void SetUp()
  {
    jsonFile_ = ":/resources/boxes.json";

    QString str(R"(
{
    "associatedActor" : "",
    "barcode" : "",
    "bbox" : {
      "x" : 1.190,
      "y" : 1.190,
      "z" : 1.050
    },
    "body_texture_urls" : [ "http://www.ac2.sg/blob_store/8c49a221.png" ],
    "body_urls" : [ "http://www.ac2.sg/blob_store/8c49a221.wrl" ],
    "boxDirection" : "",
    "cog" : {
      "x" : -0.003847535699605942,
      "y" : 0.000359053723514080,
      "z" : 0.0
    },
    "facesPath" : [
      "http://www.ac2.sg/blob_store/8c49a221_0.jpg",
      "http://www.ac2.sg/blob_store/8c49a221_1.jpg",
      "http://www.ac2.sg/blob_store/8c49a221_2.jpg",
      "http://www.ac2.sg/blob_store/8c49a221_3.jpg",
      "http://www.ac2.sg/blob_store/8c49a221_4.jpg"
    ],
    "featurePointsPath" : "http://www.ac2.sg/blob_store/8c49a221.dat",
    "fragile" : 0,
    "graspPose" : {
      "orientation" : {
        "w" : 0.0,
        "x" : 0.0,
        "y" : 0.0,
        "z" : 0.0
      },
      "position" : {
        "x" : 1.1,
        "y" : 2.2,
        "z" : 3.3
      }
    },
    "isArticulated" : 0,
    "kinematicModelPath" : "",
    "labels" : [ "flammable", "onpalette" ],
    "markerId" : "",
    "material" : "wooden",
    "measureDebugPath" : "http://www.ac2.sg/blob_store/8c49a221_measure_debug.jpg",
    "meshPath" : "http://www.ac2.sg/blob_store/8c49a221.zip",
    "modelPath" : "",
    "objectPointsPath" : "http://www.ac2.sg/blob_store/8c49a221.pcd",
    "placePose" : {
      "orientation" : {
        "w" : 0.0,
        "x" : 0.0,
        "y" : 0.0,
        "z" : 0.0
      },
      "position" : {
        "x" : 0.0,
        "y" : 0.0,
        "z" : 0.0
      }
    },
    "rectLabelVecs" : [
      null,
      [
        {
          "h" : 89,
          "label" : 4,
          "w" : 95,
          "x" : 241,
          "y" : 197
        }
      ],
      null,
      [
        {
          "h" : 98,
          "label" : 4,
          "w" : 103,
          "x" : 219,
          "y" : 203
        }
      ],
      null
    ],
    "robotTool" : "foamgripper",
    "semanticName" : "",
    "state" : "",
    "storage_location" : "",
    "targetPoseVec" : [
      {
        "orientation" : {
          "w" : 0.7088728547096252,
          "x" : 0.0,
          "y" : 0.0,
          "z" : 0.7053363323211670
        },
        "position" : {
          "x" : -0.008621707558631897,
          "y" : 0.0468456931412220,
          "z" : 0.524058461189270
        }
      }
    ],
    "thumbnailPath" : "http://www.ac2.sg/blob_store/8c49a221_thumbnail.jpg",
    "type" : "pallet",
    "uuid" : "8c49a221",
    "weight" : 39.09999847412109,
    "zone" : "measurement_area"
  }
)");
    QJsonDocument doc = QJsonDocument::fromJson(str.toUtf8());
    jsonObject_ = doc.object();
  }

  void TearDown()
  {
  }

  QString jsonFile_;
  QJsonObject jsonObject_;
};

TEST_F(BoxJsonParserTestFixture, JsonFilesToBoxesTest)
{
  std::vector<bpa::Box> boxes = BoxJsonParser::getBoxesFromJsonFile(jsonFile_);

  EXPECT_EQ(3, boxes.size());

  bpa::Box box = boxes[2];

  EXPECT_DOUBLE_EQ(0.51, box.m_length);
  EXPECT_DOUBLE_EQ(0.50, box.m_width);
  EXPECT_DOUBLE_EQ(1.120, box.m_height);
  EXPECT_DOUBLE_EQ(19.0, box.m_mass);
  EXPECT_EQ("0b99554a", box.m_name);
  EXPECT_EQ("styrofoam", box.material);
  EXPECT_EQ("foamgripper", box.tool_name);

  std::vector<std::string> labels{ "thisSideUp", "fragile", "keepDry" };
  EXPECT_EQ(labels, box.box_labels);

  EXPECT_FALSE(box.is_packed);
  EXPECT_TRUE(box.is_stackable);

  Eigen::Vector3d pos;
  pos << -0.04593759030103683, 0.04091981053352356, 0.5608614087104797;
  EXPECT_EQ(pos, box.position.position);
}

TEST_F(BoxJsonParserTestFixture, JsonObjectToBoxTest)
{
  bpa::Box box = BoxJsonParser::getBoxFromJsonObject(jsonObject_);

  EXPECT_DOUBLE_EQ(1.20, box.m_length);
  EXPECT_DOUBLE_EQ(1.20, box.m_width);
  EXPECT_DOUBLE_EQ(1.050, box.m_height);
  EXPECT_DOUBLE_EQ(39.09999847412109, box.m_mass);
  EXPECT_EQ("8c49a221", box.m_name);
  EXPECT_EQ("wooden", box.material);
  EXPECT_EQ("foamgripper", box.tool_name);

  std::vector<std::string> labels{ "flammable", "onpalette" };
  EXPECT_EQ(labels, box.box_labels);

  EXPECT_FALSE(box.is_packed);
  EXPECT_FALSE(box.is_stackable);

  Eigen::Vector3d pos;
  pos << -0.008621707558631897, 0.0468456931412220, 0.524058461189270;
  EXPECT_EQ(pos, box.position.position);
}
}