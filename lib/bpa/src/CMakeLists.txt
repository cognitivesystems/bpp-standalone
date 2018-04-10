find_package(Bullet REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(LIB_VERSION_MAJOR 0)
set(LIB_VERSION_MINOR 0)
set(LIB_VERSION_PATCH 1)
set(LIB_VERSION_STRING ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

include_directories(${BULLET_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

set(BpaLib_hdr
  Bin.h
  Box.h
  bpp_functions.h
  FittingPoint.h
  HoldingPlatform.h
  MixedScore.h
  Parameters.h
  Point.h
  pp_functions.h
  SimulatedBox.h
  PhysicsBullet.h)

set(BpaLib_src
  Bin.cpp
  Box.cpp
  bpp_functions.cpp
  FittingPoint.cpp
  HoldingPlatform.cpp
  MixedScore.cpp
  Parameters.cpp
  PhysicsBullet.cpp
  Point.cpp
  pp_functions.cpp
  PhysicsBullet.cpp)

add_library(bpa SHARED ${BpaLib_hdr} ${BpaLib_src})
set_target_properties(bpa PROPERTIES VERSION ${LIB_VERSION_STRING} SOVERSION ${LIB_VERSION_MAJOR})
set_target_properties(bpa PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_include_directories(bpa PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(bpa
  ${BULLET_DYNAMICS_LIBRARY}
  ${BULLET_COLLISION_LIBRARY}
  ${BULLET_MATH_LIBRARY}
  jsoncpp)

add_library(bpa_static STATIC ${BpaLib_hdr} ${BpaLib_src})
set_target_properties(bpa_static PROPERTIES OUTPUT_NAME "bpa")
set_target_properties(bpa_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_include_directories(bpa_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(bpa_static
  ${BULLET_DYNAMICS_LIBRARY}
  ${BULLET_COLLISION_LIBRARY}
  ${BULLET_MATH_LIBRARY}
  jsoncpp)

install(TARGETS bpa bpa_static
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/bpa
  FILES_MATCHING PATTERN "*.h")