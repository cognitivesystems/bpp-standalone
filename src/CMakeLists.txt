set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)

if (NOT CMAKE_PREFIX_PATH)
  set(CMAKE_PREFIX_PATH "/opt/Qt5/5.10.1/gcc_64/lib/cmake")
endif ()

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets 3DCore 3DExtras 3DRender 3DInput)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${BULLET_INCLUDE_DIR})

set(HEADERS
  mainwindow.h
  scenerenderer3d.h
  bppinterface.h)

set(SOURCES
  main.cpp
  mainwindow.cpp
  scenerenderer3d.cpp
  bppinterface.cpp)

add_executable(qt3dsimple ${HEADERS} ${SOURCES})
target_link_libraries(qt3dsimple
  ${BULLET_LIBRARIES}
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
  BoxFactory
  bpa)
