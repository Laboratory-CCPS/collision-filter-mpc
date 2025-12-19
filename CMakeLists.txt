cmake_minimum_required(VERSION 3.8)
project(safety_filter_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- PYTHON ---
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(ACADOS_CODEGEN_PYTHON "" CACHE FILEPATH "Python interpreter for acados code generation")
if(NOT ACADOS_CODEGEN_PYTHON)
  message(STATUS "ACADOS_CODEGEN_PYTHON not set -> fallback to $ENV{HOME}/.acados_env/bin/python")
  set(ACADOS_CODEGEN_PYTHON "$ENV{HOME}/.acados_env/bin/python")
endif()

set(NUM_OBS 10 CACHE STRING "Number of obstacles used of the occupancy grid")
set(HORIZON 15 CACHE STRING "Prediction horizon length")

set(ACADOS_GENERATED_CODE_DIR ${CMAKE_CURRENT_BINARY_DIR}/c_generated_code)
set(ACADOS_GENERATED_LIB ${ACADOS_GENERATED_CODE_DIR}/libacados_ocp_solver_agilex.so)
set(ACADOS_PYTHON_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/safety_filter_mpc.py)
set(ACADOS_GENERATE_WRAPPER  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_solver.sh)

add_custom_command(
    OUTPUT ${ACADOS_GENERATED_LIB}
    COMMAND ${ACADOS_GENERATE_WRAPPER}
            ${ACADOS_CODEGEN_PYTHON}
            ${ACADOS_PYTHON_SCRIPT}
            ${ACADOS_GENERATED_CODE_DIR}
            ${NUM_OBS}
            ${HORIZON}

    DEPENDS ${ACADOS_PYTHON_SCRIPT} ${ACADOS_GENERATE_WRAPPER}
    COMMENT "Generating ACADOS solver via wrapper script..."
    USES_TERMINAL 
)

add_custom_target(generate_acados_code
    DEPENDS ${ACADOS_GENERATED_LIB}
)

# --- ROS Abhängigkeiten ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenMP REQUIRED)

# --- ACADOS ---
set(ACADOS_SOURCE_DIR_VAR "$ENV{ACADOS_SOURCE_DIR}")
set(ACADOS_INCLUDE_PATH ${ACADOS_SOURCE_DIR_VAR}/include)
set(ACADOS_LIB_DIR ${ACADOS_SOURCE_DIR_VAR}/lib)
if(NOT EXISTS ${ACADOS_LIB_DIR})
    message(FATAL_ERROR "Acados lib directory not found: ${ACADOS_LIB_DIR}")
endif()

# --- EXECUTABLE ---
add_executable(safety_filter_mpc_node 
    src/safety_filter_mpc_node.cpp
)

add_dependencies(safety_filter_mpc_node generate_acados_code)

# Include Directories
target_include_directories(safety_filter_mpc_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ACADOS_GENERATED_CODE_DIR}
    ${ACADOS_INCLUDE_PATH}
    ${ACADOS_INCLUDE_PATH}/acados
    ${ACADOS_INCLUDE_PATH}/blasfeo/include
    ${ACADOS_INCLUDE_PATH}/hpipm/include
)
# Link Libraries
target_link_libraries(safety_filter_mpc_node PUBLIC
    ${ACADOS_GENERATED_LIB}
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    m
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
)

# Declare dependencies
ament_target_dependencies(safety_filter_mpc_node PUBLIC 
    rclcpp
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
)

# --- INSTALLATIONS ---
install(FILES 
    ${ACADOS_GENERATED_LIB}
    DESTINATION lib
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
    include/
    DESTINATION include
)

install(TARGETS 
    safety_filter_mpc_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(
        test/test_safety_filter.launch.py
        TARGET test_safety_filter
        TIMEOUT 180
    )
endif()

ament_package()