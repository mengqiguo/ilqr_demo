cmake_minimum_required(VERSION 3.10)
project(BicycleModel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp 0.8 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)



add_executable(run_bicycle 
    src/bicycle_model.cpp 
    src/generate_traj.cpp
    src/ilqr_solver.cpp
    src/main.cpp
)

target_include_directories(run_bicycle PRIVATE include)
# 链接yaml库
target_link_libraries(run_bicycle PRIVATE
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    Python3::Python
    Python3::NumPy
)
