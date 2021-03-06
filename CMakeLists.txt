cmake_minimum_required(VERSION 2.8.3)
project(qt3DExample)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-narrowing")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_PREFIX_PATH "/opt/Qt5/lib/cmake")

# include necessary qt3d modules
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets 3DCore 3DExtras 3DRender 3DInput)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIR}
    /usr/local/include/box_factory
    /usr/local/include/bpa
    #/usr/local/include/bullet
)
LINK_DIRECTORIES(
    /opt/Qt5/lib
    /usr/local/lib
)

set(HEADERS
     src/mainwindow.h
     src/scenerenderer3d.h
     src/bppinterface.h
)


set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/scenerenderer3d.cpp
    src/bppinterface.cpp
)


add_executable(qt3dsimple ${HEADERS} ${SOURCES})

# link the qt3d libraries
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
    bpa
)
