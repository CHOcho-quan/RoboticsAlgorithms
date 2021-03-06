project(CppRobotics_quan)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS ${OpenCV_LIBS})

add_definitions(-std=c++11)

set(include_dir ${PROJECT_SOURCE_DIR}/include/planning
    ${PROJECT_SOURCE_DIR}/include/optimization
    ${PROJECT_SOURCE_DIR}/include/filtering
    ${PROJECT_SOURCE_DIR}/include/3rdParty
    ${EIGEN3_INCLUDE_DIR})
include_directories(${include_dir})

add_executable(astar src/planning/a_star.cpp)
target_link_libraries(astar ${OpenCV_LIBS} )

add_executable(dstar src/planning/d_star.cpp)
target_link_libraries(dstar ${OpenCV_LIBS} )

add_executable(rrt src/planning/rrt.cpp)
target_link_libraries(rrt ${OpenCV_LIBS} )

add_executable(rrt_star src/planning/rrt_star.cpp)
target_link_libraries(rrt_star ${OpenCV_LIBS} )

add_executable(prm src/planning/prm.cpp)
target_link_libraries(prm ${OpenCV_LIBS} )

add_executable(kf src/filtering/kalman_filter.cpp)
target_link_libraries(kf ${OpenCV_LIBS} )

add_executable(ekf src/filtering/ekf.cpp)
target_link_libraries(ekf ${OpenCV_LIBS} )

file(GLOB ALGLIB_SOURCES include/3rdParty/ALGLIB/*.cpp )
file(GLOB ALGLIB_HEADERS include/3rdParty/ALGLIB/*.h )
add_executable(m_snap src/optimization/minimum_snap_control.cpp ${ALGLIB_HEADERS} ${ALGLIB_SOURCES})
target_link_libraries(m_snap ${OpenCV_LIBS})
