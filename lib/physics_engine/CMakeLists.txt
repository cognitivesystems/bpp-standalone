find_package(Bullet REQUIRED)

set(LIB_VERSION_MAJOR 0)
set(LIB_VERSION_MINOR 1)
set(LIB_VERSION_PATCH 0)
set(LIB_VERSION_STRING ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
include_directories(include ${BULLET_INCLUDE_DIR})

set(PhysicsEngineLib_hdr
  include/PhysicsEngine.h)

set(PhysicsEngineLib_src
  src/PhysicsEngine.cpp)

add_library(PhysicsEngine SHARED ${PhysicsEngineLib_hdr} ${PhysicsEngineLib_src})
set_target_properties(PhysicsEngine PROPERTIES VERSION ${LIB_VERSION_STRING} SOVERSION ${LIB_VERSION_MAJOR})
set_target_properties(PhysicsEngine PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_include_directories(PhysicsEngine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(PhysicsEngine
  ${BULLET_DYNAMICS_LIBRARY}
  ${BULLET_COLLISION_LIBRARY}
  ${BULLET_MATH_LIBRARY})

add_library(PhysicsEngine_static STATIC ${PhysicsEngineLib_hdr} ${PhysicsEngineLib_src})
set_target_properties(PhysicsEngine_static PROPERTIES OUTPUT_NAME "PhysicsEngine")
set_target_properties(PhysicsEngine_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_include_directories(PhysicsEngine_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(PhysicsEngine_static
  ${BULLET_DYNAMICS_LIBRARY}
  ${BULLET_COLLISION_LIBRARY}
  ${BULLET_MATH_LIBRARY})

add_subdirectory(test)

install(TARGETS PhysicsEngine PhysicsEngine_static
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include/physics_engine
  FILES_MATCHING PATTERN "*.h")