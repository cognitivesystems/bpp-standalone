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

set(BpaTest_src
  BoxJsonParserTest.cpp
  BulletPhysicsTest.cpp
  resources/boxes.qrc)
add_executable(bpa-test ${BpaTest_src})
target_link_libraries(bpa-test
  bpa
  gtest
  gtest_main
  Qt5::Core)
add_test(NAME BpaTest
  COMMAND bpa-test
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
