find_package(Eigen3 REQUIRED)
find_package(Qt5Core REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(${EIGEN3_INCLUDE_DIR})

set(BoxFactoryTest_src BoxJsonParserTest.cpp resources/boxes.qrc)
add_executable(BoxFactory-test ${BoxFactoryTest_src})
target_link_libraries(BoxFactory-test
  BoxFactory
  gtest
  gtest_main
  Qt5::Core)
add_test(NAME BoxFactoryTest
  COMMAND BoxFactory-test
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})