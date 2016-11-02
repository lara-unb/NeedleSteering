cmake_minimum_required(VERSION 2.8)
project(planner)

find_package(OpenCV REQUIRED)
find_package(OMPL REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
# Use ${EIGEN3_...} variables in either case below
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

message("EIGEN include dirs:" ${EIGEN3_INCLUDE_DIR})
message("OMPL include dirs:" ${OMPL_INCLUDE_DIRS})
message("OpenCV include dirs:" ${OpenCV_INCLUDE_DIRS})
message("Boost include dirs:" ${Boost_INCLUDE_DIRS})

# message("OMPL library dirs:" ${OMPL_LIBRARIES})
# message("Boost library dirs:" ${Boost_LIBRARIES})
# message("OpenCV library dirs:" ${OpenCV_LIBS})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

add_executable(planner src/planner.cpp)
target_link_libraries(planner ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
