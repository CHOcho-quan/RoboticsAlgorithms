project(CppRobotics_quan)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(./include)

add_executable(astar src/a_star.cpp)
target_link_libraries(astar ${OpenCV_LIBS} )

add_executable(rrt src/rrt.cpp)
target_link_libraries(rrt ${OpenCV_LIBS} )
