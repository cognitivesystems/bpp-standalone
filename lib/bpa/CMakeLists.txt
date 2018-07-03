find_package(Bullet REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Qt5Core REQUIRED)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

set(LIB_VERSION_MAJOR 0)
set(LIB_VERSION_MINOR 1)
set(LIB_VERSION_PATCH 0)
set(LIB_VERSION_STRING ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

include_directories(include ${BULLET_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

set(BpaLib_hdr
  include/Bin.h
  include/Box.h
  include/bpp_functions.h
  include/FittingPoint.h
  include/HoldingPlatform.h
  include/MixedScore.h
  include/Parameters.h
  include/Point.h
  include/pp_functions.h
  include/SimulatedBox.h
  include/PhysicsBullet.h
  include/BoxJsonParser.h
  include/BulletPhysics.h
  include/KineoPhysics.h
  include/PhysicsEngine.h)

set(BpaLib_src
  src/Bin.cpp
  src/Box.cpp
  src/bpp_functions.cpp
  src/FittingPoint.cpp
  src/HoldingPlatform.cpp
  src/MixedScore.cpp
  src/Parameters.cpp
  src/PhysicsBullet.cpp
  src/Point.cpp
  src/pp_functions.cpp
  src/PhysicsBullet.cpp
  src/BoxJsonParser.cpp
  src/BulletPhysics.cpp
  src/KineoPhysics.cpp
  src/PhysicsEngine.cpp)

add_library(bpa SHARED ${BpaLib_hdr} ${BpaLib_src})
set_target_properties(bpa PROPERTIES VERSION ${LIB_VERSION_STRING} SOVERSION ${LIB_VERSION_MAJOR})
set_target_properties(bpa PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_include_directories(bpa PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(bpa
  ${BULLET_DYNAMICS_LIBRARY}
  ${BULLET_COLLISION_LIBRARY}
  ${BULLET_MATH_LIBRARY}
  jsoncpp
  Qt5::Core)

add_library(bpa_static STATIC ${BpaLib_hdr} ${BpaLib_src})
set_target_properties(bpa_static PROPERTIES OUTPUT_NAME "bpa")
set_target_properties(bpa_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_include_directories(bpa_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(bpa_static
  ${BULLET_DYNAMICS_LIBRARY}
  ${BULLET_COLLISION_LIBRARY}
  ${BULLET_MATH_LIBRARY}
  jsoncpp
  Qt5::Core)

add_subdirectory(test)

install(TARGETS bpa bpa_static
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include/bpa
  FILES_MATCHING PATTERN "*.h")