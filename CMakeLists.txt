project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp
        src/TrajectoryGenerator/CostFunctions/CostFunction.h
        src/TrajectoryGenerator/CostFunctions/CollisionCostFunction.h
        src/TrajectoryGenerator/CostFunctions/BufferCostFunction.h
        src/TrajectoryGenerator/CostFunctions/MaxAccelerationCostFunction.h
        src/TrajectoryGenerator/CostFunctions/TotalAccelerationCostFunction.h
        src/TrajectoryGenerator/CostFunctions/MaxJerkCostFunction.h
        src/TrajectoryGenerator/CostFunctions/TotalJerkCostFunction.h
        src/TrajectoryGenerator/CostFunctions/EfficiencyCostFunction.h
        src/Vehicle.cpp
        src/Vehicle.h
        src/TrajectoryGenerator/TrajectoryGenerator.cpp
        src/TrajectoryGenerator/TrajectoryGenerator.h
        src/TrajectoryGenerator/DriveStraightTrajectoryGenerator.cpp
        src/TrajectoryGenerator/DriveStraightTrajectoryGenerator.h
        src/TrajectoryGenerator/JMTSolver.cpp
        src/TrajectoryGenerator/JMTSolver.h
        src/spline.h
        src/PathConverter.cpp
        src/PathConverter.h
        src/TelemetryHandler.cpp
        src/TelemetryHandler.h src/TrajectoryGenerator/Trajectory.h src/TrajectoryGenerator/CostFunctions/DDiffCostFunction.h src/TrajectoryGenerator/CostFunctions/SDiffCostFunction.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories("src/include")
include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
