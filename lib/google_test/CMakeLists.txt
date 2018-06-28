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