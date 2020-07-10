cmake_minimum_required(VERSION 3.10)
project(CBSH2)
set (CMAKE_CXX_STANDARD 17)


add_subdirectory(library/yaml-cpp)

add_library(cbsh2 INTERFACE)
target_include_directories(
  cbsh2
  INTERFACE 
  include
  library/yaml-cpp/include
 )

file(GLOB SOURCES "include/CBSH2/*.cpp")
file(GLOB DIRECTIVES "include/CBSH2/*.h")


# Find Boost
find_package( Boost 1.58 REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )

add_executable(CBSH2 example/cbsh_test.cpp ${SOURCES} ${DIRECTIVES})
target_link_libraries(CBSH2 cbsh2 ${Boost_LIBRARIES} yaml-cpp)