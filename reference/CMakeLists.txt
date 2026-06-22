cmake_minimum_required(VERSION 3.20)
project(caliburn-reference LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Eigen via FetchContent ---
include(FetchContent)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

enable_testing()

# --- Integrators ---
add_executable(test_rk4
    integrators/rk4.cpp
    integrators/test_rk4.cpp
)
target_include_directories(test_rk4 PRIVATE integrators)
target_link_libraries(test_rk4 PRIVATE Eigen3::Eigen)
add_test(NAME rk4 COMMAND test_rk4)

# --- Controllers ---
add_executable(test_pid
    controllers/pid.cpp
    controllers/test_pid.cpp
)
target_include_directories(test_pid PRIVATE controllers)
add_test(NAME pid COMMAND test_pid)

add_executable(test_lqr
    controllers/lqr.cpp
    controllers/test_lqr.cpp
)
target_include_directories(test_lqr PRIVATE controllers)
target_link_libraries(test_lqr PRIVATE Eigen3::Eigen)
add_test(NAME lqr COMMAND test_lqr)

# --- Observers ---
add_executable(test_kalman
    observers/kalman_filter.cpp
    observers/test_kalman.cpp
)
target_include_directories(test_kalman PRIVATE observers)
target_link_libraries(test_kalman PRIVATE Eigen3::Eigen)
add_test(NAME kalman COMMAND test_kalman)

# --- Simulation ---
add_executable(test_fixed_timestep
    simulation/fixed_timestep.cpp
    simulation/test_fixed_timestep.cpp
)
target_include_directories(test_fixed_timestep PRIVATE simulation)
add_test(NAME fixed_timestep COMMAND test_fixed_timestep)

add_executable(test_sim_loop
    simulation/fixed_timestep.cpp
    simulation/sim_loop.cpp
    simulation/test_sim_loop.cpp
)
target_include_directories(test_sim_loop PRIVATE simulation)
add_test(NAME sim_loop COMMAND test_sim_loop)
