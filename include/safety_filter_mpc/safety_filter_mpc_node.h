#ifndef SAFETY_FILTER_MPC_NODE_H
#define SAFETY_FILTER_MPC_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <Eigen/Dense>
#include <thread>
#include <functional>
#include <unordered_map>
#include <memory>
#include <atomic>

// ROS2 msgs
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// TF
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// Acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados/ocp_nlp/ocp_nlp_sqp_rti.h"
#include "acados/ocp_nlp/ocp_nlp_common.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "blasfeo_d_aux_ext_dep.h"
#include "acados_solver_agilex.h"

// Package
#include "safety_filter_mpc/utils.hpp"
#include "safety_filter_mpc/config.hpp"
#include "safety_filter_mpc/marker_publisher.hpp"
#include "safety_filter_mpc/obstacle_extract.hpp"

#define SQRT2 1.4142135623730951
#define EPSILON 0.000001
#define MAX_NUM_OBSTACLES (AGILEX_NH / 2)

namespace safety_filter_mpc
{

class MPCSafetyFilterNode : public rclcpp::Node {
private:
    // --- ROS2 Subscriber and Publisher ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr unsafe_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;

    // --- ROS2 Parameters
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    using ParamHandler = std::function<void(const rclcpp::Parameter&, rcl_interfaces::msg::SetParametersResult&)>;
    std::unordered_map<std::string, ParamHandler> parameter_handlers_;
    
    // --- ROS2 TF and Timer ---
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::chrono::steady_clock::time_point last_unsafe_vel_time_;

    // --- ROS Callbacks Groups ---
    rclcpp::CallbackGroup::SharedPtr timer_cbg_;
    rclcpp::CallbackGroup::SharedPtr services_cbg_;

    // --- Acados Solver ---
    agilex_solver_capsule *ocp_capsule_;
    ocp_nlp_config* ocp_nlp_config_;
    ocp_nlp_dims* ocp_nlp_dims_;
    ocp_nlp_in* ocp_nlp_in_;
    ocp_nlp_out* ocp_nlp_out_;
    void* ocp_nlp_opts_;

    // --- Configs ---
    SafetyFilterNodeConfig config_;
    SurfaceBoundaryParams sb_;

    // --- Mutex ---
    std::mutex data_mutex_;
    std::mutex tf_mutex_;
    std::recursive_mutex solver_mutex_;

    // --- Data ---
    std::string map_frame_ = "map";
    std::string odom_frame_ = "odom";

    std::atomic_bool odom_received_{false}, map_received_{false}, u_received_{false};
    std::atomic<int> last_num_obstacles_{-1};
    std::atomic<double> squared_safety_radius_{0.0}; // (map_resolution * sqrt(2) + bot_size)^2
    std::atomic<double> last_map_resolution_{0.0};
    
    bool first_solve_{true};
    std::array<double, AGILEX_NU> u0_default_;
    std::array<double, AGILEX_NU> current_unsafe_u_;
    std::array<double, AGILEX_NX> current_x_;
    std::array<double, AGILEX_NP> current_p_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
    std::vector<Eigen::Vector2d> latest_obstacles_;
    geometry_msgs::msg::TransformStamped last_known_tf_;

    double last_dyn_lbu0_{0.0};
    double last_dyn_ubu0_{0.0};
    

public:
    MPCSafetyFilterNode();
    ~MPCSafetyFilterNode();

private:
    // --- Core Methods ---
    void initialize_solver();
    void control_loop();
    int prepare_rti_solve();
    int feedback_rti_solve();
    int full_rti_solve();

    // --- ROS Callbacks ---
    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg);
    void map_callback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void cmd_vel_unsafe_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg);

    // --- ROS Publisher ---
    void publish_input(const double* u0);

    // --- Parameter Handling Methods ----
    void setup_parameter_handlers();
    void declare_parameters();
    void load_parameters();
    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params);
    
    template<size_t N>
    void update_constraint(
        const rclcpp::Parameter& param,
        rcl_interfaces::msg::SetParametersResult& result,
        const char* field,
        const std::vector<int>& stages);
    template<size_t N>
    void update_cost(
        const rclcpp::Parameter& param,
        rcl_interfaces::msg::SetParametersResult& result,
        const char* field,
        const std::vector<int>& stages);

    // --- ROS Visualizer ---
    void visualize_parameters(
        const std::vector<Eigen::Vector2d>& obstacles);
    void visualize_trajectory();

    // --- Helper & Utility Methods ---
    void start_control_timer(double period_seconds = 0.02);
    void solver_status_behaviour(
        const int status, 
        const double unsafe_vel);
    void calc_squared_safety_radius();
    std::array<double, AGILEX_NP> get_p_values(
        const std::vector<Eigen::Vector2d>& obstacles);
    std::array<double, AGILEX_NH> get_lh_values(
        const std::vector<Eigen::Vector2d>& obstacles);

    // --- acados getter --- 
    void get_input(double* u, int stage);
    void get_state(double* x, int stage);

    // --- acados setter --- 
    void set_x0(double* x0);
    void set_yref0(double* yref0);
    void set_lh(double* lh);
    void set_ocp_parameters(double* p, size_t np);
    void set_control_bound(double max_velocity);
    void warmstart_solver_states(double* x0);
    std::vector<Eigen::Vector2d> set_solver_obstacles(
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr map, 
        double* x0);
};

} // namespace safety_filter_mpc

#endif // SAFETY_FILTER_MPC_NODE_H