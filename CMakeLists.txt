cmake_minimum_required(VERSION 3.2)
project(qt3DExample)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

if (NOT CMAKE_PREFIX_PATH)
  set(CMAKE_PREFIX_PATH "/opt/Qt5/lib/cmake")
endif ()

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets 3DCore 3DExtras 3DRender 3DInput Charts DataVisualization)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${BULLET_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR})

set(qt3dsimple_hdr
  include/mainwindow.h
  include/bppinterface.h
  include/View/BoxEntity.h
  include/View/PlaneEntity.h
  include/opt/defs.h
  include/opt/ParticleFilter.h
  include/opt/ParamEstimator.h
  include/opt/pdflib.hpp
  include/opt/rnglib.hpp
  include/Model/BoxSet.h
  include/View/DashboardView.h
  include/View/BinPackingView.h
  include/ViewMgr/BinPackingViewMgr.h
  include/View/DistributionChart.h
  include/View/ObjectiveChart.h
  include/ViewMgr/DistributionChartViewMgr.h
  include/MathUtils.h)

set(qt3dsimple_src
  src/main.cpp
  src/mainwindow.cpp
  src/bppinterface.cpp
  src/View/BoxEntity.cpp
  src/View/PlaneEntity.cpp
  src/opt/ParticleFilter.cpp
  src/opt/ParamEstimator.cpp
  src/opt/pdflib.cpp
  src/opt/rnglib.cpp
  src/Model/BoxSet.cpp
  src/View/DashboardView.cpp
  src/View/BinPackingView.cpp
  src/ViewMgr/BinPackingViewMgr.cpp
  src/View/DistributionChart.cpp
  src/View/ObjectiveChart.cpp
  src/ViewMgr/DistributionChartViewMgr.cpp
  src/MathUtils.cpp)

set(qt3dsimple_ui
  include/mainwindow.ui
  include/View/DistributionChart.ui
  include/View/ObjectiveChart.ui
  include/View/DashboardView.ui)

set(qt3dsimple_resource
  src/resources/boxes.qrc)

add_executable(qt3dsimple ${qt3dsimple_hdr} ${qt3dsimple_src} ${qt3dsimple_ui} ${qt3dsimple_resource})
target_link_libraries(qt3dsimple
  ${BULLET_LIBRARIES}
  ${Boost_LIBRARIES}
  #${BULLET_DYNAMICS_LIBRARY}
  #${BULLET_COLLISION_LIBRARY}
  #/usr/local/lib/libLinearMath.a
  #/usr/local/lib/libBulletDynamics.a
  #/usr/local/lib/libBulletCollision.a
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::3DCore
  Qt5::3DExtras
  Qt5::3DRender
  Qt5::3DInput
  Qt5::Charts
  Qt5::DataVisualization
  BoxFactory
  bpa)

add_executable(testobj src/testobj.cpp)
target_link_libraries(testobj
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::3DCore
  Qt5::3DExtras
  Qt5::3DRender
  Qt5::3DInput)


add_subdirectory(lib)
