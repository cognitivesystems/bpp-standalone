cmake_minimum_required(VERSION 2.8.3)
project(qt3DExample)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-narrowing")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_PREFIX_PATH "/opt/Qt5/lib/cmake")

# include necessary qt3d modules
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets 3DCore 3DExtras 3DRender 3DInput)
#find_package(Qt5Core)
#find_package(Qt5Gui)
#find_package(Qt5Widgets)
#find_package(Qt53DCore)
#find_package(Qt53DExtras)
#find_package(Qt53DRender)
#find_package(Qt53DInput)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
LINK_DIRECTORIES(
    /opt/Qt5/lib

)

set(HEADERS
     src/mainwindow.h
     src/scenerenderer3d.h
)


set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/scenerenderer3d.cpp
)


add_executable(qt3dsimple ${HEADERS} ${SOURCES})

# link the qt3d libraries
target_link_libraries(qt3dsimple
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::3DCore
    Qt5::3DExtras
    Qt5::3DRender
    Qt5::3DInput
    )
