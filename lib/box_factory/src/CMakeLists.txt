find_package(Eigen3 REQUIRED)
find_package(Qt5Core REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

set(LIB_VERSION_MAJOR 0)
set(LIB_VERSION_MINOR 0)
set(LIB_VERSION_PATCH 1)
set(LIB_VERSION_STRING ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

include_directories(${EIGEN3_INCLUDE_DIR})

set(BoxFactoryLib_hdr
  BoxJsonParser.h)

set(BoxFactoryLib_src
  BoxJsonParser.cpp)

add_library(BoxFactory SHARED ${BoxFactoryLib_hdr} ${BoxFactoryLib_src})
set_target_properties(BoxFactory PROPERTIES VERSION ${LIB_VERSION_STRING} SOVERSION ${LIB_VERSION_MAJOR})
set_target_properties(BoxFactory PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_include_directories(BoxFactory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(BoxFactory
  bpa
  Qt5::Core)

add_library(BoxFactory_static STATIC ${BoxFactoryLib_hdr} ${BoxFactoryLib_src})
set_target_properties(BoxFactory_static PROPERTIES OUTPUT_NAME "BoxFactory")
set_target_properties(BoxFactory_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_include_directories(BoxFactory_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(BoxFactory_static
  bpa
  Qt5::Core)

install(TARGETS BoxFactory BoxFactory_static
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/box_factory
  FILES_MATCHING PATTERN "*.h")