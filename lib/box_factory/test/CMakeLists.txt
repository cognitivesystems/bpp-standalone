find_package(Eigen3 REQUIRED)
find_package(Qt5Core REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(${EIGEN3_INCLUDE_DIR})

enable_testing()

set(3rdPartyDir ${PROJECT_BINARY_DIR}/3rdparty)

configure_file(CMakeLists.txt.in ${3rdPartyDir}/googletest-download/CMakeLists.txt)
execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${3rdPartyDir}/googletest-download")
execute_process(COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${3rdPartyDir}/googletest-download")

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

add_subdirectory("${3rdPartyDir}/googletest-src"
  "${3rdPartyDir}/googletest-build")

set(BoxFactoryTest_src BoxJsonParserTest.cpp resources.qrc)
add_executable(BoxFactory-test ${BoxFactoryTest_src})
target_link_libraries(BoxFactory-test
  BoxFactory
  gtest
  gtest_main
  Qt5::Core)
add_test(NAME BoxFactoryTest
  COMMAND BoxFactory-test
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})