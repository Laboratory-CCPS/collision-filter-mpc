#include "safety_filter_mpc/safety_filter_mpc_node.h"

namespace safety_filter_mpc
{

MPCSafetyFilterNode::MPCSafetyFilterNode()
    : Node("safety_filter_mpc_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing MPC Safety Filter Node...");

    // --- default values ---
    config_ = SafetyFilterNodeConfig();
    sb_ = SurfaceBoundaryParams();
    u0_default_.fill(0.0);
    current_unsafe_u_.fill(0.0);
    current_x_.fill(0.0);
    current_p_.fill(0.0);
    last_unsafe_vel_time_ = std::chrono::steady_clock::now();

    // --- Callback Groups ---
    timer_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    services_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // --- Parameters ---
    this->setup_parameter_handlers();
    this->declare_parameters();
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MPCSafetyFilterNode::on_parameter_update, this, std::placeholders::_1)
    );

    // --- Subscriber ---
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = services_cbg_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MPCSafetyFilterNode::odom_callback, this, std::placeholders::_1), sub_options);

    unsafe_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_unsafe", 10,
        std::bind(&MPCSafetyFilterNode::cmd_vel_unsafe_callback, this, std::placeholders::_1), sub_options);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local(), // transient_local für die Karte
        std::bind(&MPCSafetyFilterNode::map_callback, this, std::placeholders::_1), sub_options);

    // --- Publisher ---
    safe_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);
    obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/safety_filter_marker/obstacles", 10);
    trajectory_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/safety_filter_marker/trajectory", 10);

    // --- TF ---
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- Init solver ---
    this->initialize_solver();
    this->load_parameters();
    this->start_control_timer(config_.ts);
}

MPCSafetyFilterNode::~MPCSafetyFilterNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down and freeing Acados solver memory.");
    if (control_timer_) control_timer_->cancel();

    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    if (ocp_capsule_) {
        int status = agilex_acados_free(ocp_capsule_);
        if (status) {
            RCLCPP_ERROR(this->get_logger(), "agilex_acados_free() returned status %d.", status);
        }
        status = agilex_acados_free_capsule(ocp_capsule_);
        if (status) {
            RCLCPP_ERROR(this->get_logger(), "agilex_acados_free_capsule() returned status %d.", status);
        }
    }
}


// --- Core Methods ---
void MPCSafetyFilterNode::initialize_solver() {
    ocp_capsule_ = agilex_acados_create_capsule();
    int status = agilex_acados_create(ocp_capsule_);
    if (status) {
        RCLCPP_FATAL(this->get_logger(), "acados_create() failed with status %d.", status);
        rclcpp::shutdown();
    }

    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    ocp_nlp_config_ = agilex_acados_get_nlp_config(ocp_capsule_);
    ocp_nlp_dims_ = agilex_acados_get_nlp_dims(ocp_capsule_);
    ocp_nlp_in_ = agilex_acados_get_nlp_in(ocp_capsule_);
    ocp_nlp_out_ = agilex_acados_get_nlp_out(ocp_capsule_);
    ocp_nlp_opts_ = agilex_acados_get_nlp_opts(ocp_capsule_);

    RCLCPP_INFO(this->get_logger(), "Acados solver initialized successfully.");
}

void MPCSafetyFilterNode::control_loop() {
    auto t_start = std::chrono::steady_clock::now();

    if (!odom_received_.load() || !map_received_.load() || !u_received_.load()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(),  *this->get_clock(), 5000,
            "Wait for initial data... [Odom: %d, Map: %d, Input: %d]",
            odom_received_.load(), map_received_.load(), u_received_.load());
        return;
    }
    
    std::array<double, AGILEX_NX> x0{};
    std::array<double, AGILEX_NY0> yref0{};
    std::vector<Eigen::Vector2d> obstacles;
    double time_since_last_vel;

    {
        std::scoped_lock lock(data_mutex_);
        yref0[0] = current_unsafe_u_[0];
        yref0[1] = current_unsafe_u_[1];
        x0 = current_x_;
        obstacles = latest_obstacles_;
        time_since_last_vel = std::chrono::duration<double>(t_start - last_unsafe_vel_time_).count();
    }

    // Safe call if no cmd_vel has been published
    if (time_since_last_vel > config_.timeout) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No unsafe velocity command received for %.2f s. Publishing zero velocity.", time_since_last_vel);
        this->publish_input(u0_default_.data());
        return;
    }

    // Prevent unneccessary computation
    if (yref0[0] == 0.0 && yref0[1] == 0.0) {
        this->publish_input(u0_default_.data());
        return;
    }

    // Get obstacles and parameters
    auto p = this->get_p_values(obstacles);
    bool update_lh = static_cast<size_t>(last_num_obstacles_.load()) != obstacles.size();
    std::array<double, AGILEX_NH> h;
    if (update_lh) {
        h = this->get_lh_values(obstacles);
    } 

    {
        std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
        this->set_x0(x0.data());
        this->set_yref0(yref0.data());
        this->set_control_bound(yref0[0]);
        this->set_ocp_parameters(p.data(), p.size());
        if (update_lh) {
            this->set_lh(h.data());
        }

        // Solve OCP
        int status;
        if (first_solve_) {
            this->warmstart_solver_states(x0.data());
            status = this->full_rti_solve();
        }
        else {
            status = this->feedback_rti_solve();
        }

        this->solver_status_behaviour(status, yref0[0]);
    }

    // Time dependent behaviour
    auto t_end = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(t_end - t_start).count();
    if (elapsed_s < config_.ts) {
        this->visualize_trajectory();
    } else {
        RCLCPP_WARN(this->get_logger(), "control_loop took %.3f s > Ts (%.3f s). Consider increasing Ts or reducing workload.",
                    elapsed_s, config_.ts);
    }
}

int MPCSafetyFilterNode::prepare_rti_solve() {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    int phase = PREPARATION;
    ocp_nlp_sqp_rti_opts_set(ocp_nlp_config_, ocp_nlp_opts_, "rti_phase", &phase);
    int status = agilex_acados_solve(ocp_capsule_);
    if (status != ACADOS_SUCCESS && status != ACADOS_READY) {
        RCLCPP_ERROR(this->get_logger(), "Solver failed at preperation phase: %d", status);
    }
    return status;
}

int MPCSafetyFilterNode::feedback_rti_solve() {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    int phase = FEEDBACK;
    ocp_nlp_sqp_rti_opts_set(ocp_nlp_config_, ocp_nlp_opts_, "rti_phase", &phase);
    int status = agilex_acados_solve(ocp_capsule_);
    if (status != ACADOS_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Solver failed at feedback phase: %d", status);
    }
    return status;
}

int MPCSafetyFilterNode::full_rti_solve() {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    int phase = PREPARATION_AND_FEEDBACK;
    ocp_nlp_sqp_rti_opts_set(ocp_nlp_config_, ocp_nlp_opts_, "rti_phase", &phase);
    int status = agilex_acados_solve(ocp_capsule_);
    if (status != ACADOS_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Solver failed with status: %d", status);
    }
    return status;
}


// --- ROS Callbacks ---
void MPCSafetyFilterNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Odom-Yaw
    const auto& q_odom = msg->pose.pose.orientation;
    const double yaw_odom = tf2::getYaw(q_odom);

    // Odom-Pose
    const double x_odom = msg->pose.pose.position.x;
    const double y_odom = msg->pose.pose.position.y;

    // TF
    geometry_msgs::msg::TransformStamped current_tf;
    bool transform_available = false;

    try {
        current_tf = tf_buffer_->lookupTransform(map_frame_, odom_frame_, msg->header.stamp, std::chrono::milliseconds(50));
        transform_available = true;
    } catch (const tf2::TransformException& e) {
        // Fallback get newest
        try {
            current_tf = tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero, std::chrono::milliseconds(50));
            transform_available = true;
        } catch (const tf2::TransformException& e2) {
            // Fallback get last
            std::lock_guard<std::mutex> lock(tf_mutex_);
            if (!last_known_tf_.header.frame_id.empty()) {
                current_tf = last_known_tf_;
                transform_available = true;
            }
        }
    }

    if (transform_available) {
        std::lock_guard<std::mutex> lock(tf_mutex_);
        last_known_tf_ = current_tf;
    } else {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "TF lookup failed completely and no fallback available. State estimation is faulty!");
        return;
    }

    const double yaw_tf = tf2::getYaw(current_tf.transform.rotation);
    const double c = std::cos(yaw_tf), s = std::sin(yaw_tf);

    const double x_map = current_tf.transform.translation.x + c * x_odom - s * y_odom;
    const double y_map = current_tf.transform.translation.y + s * x_odom + c * y_odom;
    const double yaw_map = std::atan2(std::sin(yaw_odom + yaw_tf), std::cos(yaw_odom + yaw_tf));

    // Assign values and get map
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    {
        std::scoped_lock lock(data_mutex_);
        current_x_[0] = x_map;
        current_x_[1] = y_map;
        current_x_[2] = yaw_map;
        current_x_[3] = msg->twist.twist.linear.x;
        current_x_[4] = msg->twist.twist.angular.z;
        map = map_;
        if (!odom_received_.load()) odom_received_.store(true);
    }

    // Recalculate Obstacles
    if (map_received_.load()) {
        Eigen::Vector2d robot_position = {x_map, y_map};
        auto obstacles = safety_filter_mpc::get_surface_boundary_obstacles(map_, robot_position, sb_);
        this->visualize_parameters(obstacles);

        {
            std::scoped_lock lock(data_mutex_);
            latest_obstacles_ = obstacles;
        }
    }
}

void MPCSafetyFilterNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_ = msg;
    if (std::fabs(last_map_resolution_.load() - msg->info.resolution) > EPSILON) {
        sb_.finalize(msg->info.resolution);
        last_map_resolution_.store(msg->info.resolution);
        last_num_obstacles_.store(-1); // Reset obstacle count so that it sets again with new safety radius
        this->calc_squared_safety_radius();
    }
    if (!map_received_.load()) map_received_.store(true);
}

void MPCSafetyFilterNode::cmd_vel_unsafe_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_unsafe_u_[0] = msg->linear.x;
    current_unsafe_u_[1] = msg->angular.z;
    last_unsafe_vel_time_ = std::chrono::steady_clock::now();
    if (!u_received_.load()) u_received_.store(true);
}


// --- ROS Publisher ---
void MPCSafetyFilterNode::publish_input(const double* u0) {
    auto safe_cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
    safe_cmd_msg->linear.x = std::clamp(u0[0], config_.constraints.lbu[0], config_.constraints.ubu[0]);
    safe_cmd_msg->angular.z = std::clamp(u0[1], config_.constraints.lbu[1], config_.constraints.ubu[1]);
    safe_vel_pub_->publish(std::move(safe_cmd_msg));
}


// --- Parameter Handling Methods ---
void MPCSafetyFilterNode::setup_parameter_handlers() {
    std::string safety_filter_prefix = "safety_filter";
    // Parameters
    parameter_handlers_[safety_filter_prefix + ".collisions.radius"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult&) {
            config_.collisions.radius = p.as_double();
            this->calc_squared_safety_radius();
        };
    parameter_handlers_[safety_filter_prefix + ".collisions.front_offset"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult&) {
            config_.collisions.front_offset = p.as_double();
        };
    parameter_handlers_[safety_filter_prefix + ".collisions.back_offset"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult&) {
            config_.collisions.back_offset = p.as_double();
        };

        // Stage Constraints
    parameter_handlers_[safety_filter_prefix + ".constraints.lbu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            update_param_array(p, config_.constraints.lbu, res);
        };
    parameter_handlers_[safety_filter_prefix + ".constraints.lbx"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, AGILEX_N);
            this->update_constraint<AGILEX_NBX>(p, res, "lbx", stages);
        };
    parameter_handlers_[safety_filter_prefix + ".constraints.lsh"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, AGILEX_N);
            this->update_constraint<AGILEX_NSH>(p, res, "lsh", stages);
        };
    parameter_handlers_[safety_filter_prefix + ".constraints.ubu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            update_param_array(p, config_.constraints.ubu, res);
        };
    parameter_handlers_[safety_filter_prefix + ".constraints.ubx"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, AGILEX_N);
            this->update_constraint<AGILEX_NBX>(p, res, "ubx", stages);
        };
    parameter_handlers_[safety_filter_prefix + ".constraints.uh"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, AGILEX_N);
            this->update_constraint<AGILEX_NH>(p, res, "uh", stages);
        };
    parameter_handlers_[safety_filter_prefix + ".constraints.ush"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, AGILEX_N);
            this->update_constraint<AGILEX_NSH>(p, res, "ush", stages);
        };

    // Weights
    parameter_handlers_[safety_filter_prefix + ".cost.W_0"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<AGILEX_NY0>(p, res, "W", std::vector<int>{0});
        };
    parameter_handlers_[safety_filter_prefix + ".cost.W"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, AGILEX_N);
            this->update_cost<AGILEX_NY>(p, res, "W", stages);
        };
    // Stage Slacks
    parameter_handlers_[safety_filter_prefix + ".cost.Zl"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, AGILEX_N);
            this->update_cost<AGILEX_NS>(p, res, "Zl", stages);
        };
    parameter_handlers_[safety_filter_prefix + ".cost.Zu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, AGILEX_N);
            this->update_cost<AGILEX_NS>(p, res, "Zu", stages);
        };
    parameter_handlers_[safety_filter_prefix + ".cost.zl"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, AGILEX_N);
            this->update_cost<AGILEX_NS>(p, res, "zl", stages);
        };
    parameter_handlers_[safety_filter_prefix + ".cost.zu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, AGILEX_N);
            this->update_cost<AGILEX_NS>(p, res, "zu", stages);
        };

    // Others
    parameter_handlers_[safety_filter_prefix + ".ts"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult&) {
            config_.ts = p.as_double();
            this->start_control_timer(config_.ts);
        };
    parameter_handlers_[safety_filter_prefix + ".timeout"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult&) {
            config_.timeout = p.as_double();
        };
}

void MPCSafetyFilterNode::declare_parameters() {
    // Constraints
    std::string safety_filter_prefix = "safety_filter";
    this->declare_parameter(safety_filter_prefix + ".collisions.radius", 0.2);
    this->declare_parameter(safety_filter_prefix + ".collisions.front_offset", 0.1);
    this->declare_parameter(safety_filter_prefix + ".collisions.back_offset", 0.1);
    this->declare_parameter(safety_filter_prefix + ".constraints.lbx", std::vector<double>{ -1.0, -1.5 });
    this->declare_parameter(safety_filter_prefix + ".constraints.ubx", std::vector<double>{ 1.0, 1.5 });
    this->declare_parameter(safety_filter_prefix + ".constraints.lbu", std::vector<double>{ -1.0, -1.5 });
    this->declare_parameter(safety_filter_prefix + ".constraints.ubu", std::vector<double>{ 1.0, 1.5 });
    this->declare_parameter(safety_filter_prefix + ".constraints.uh", std::vector<double>{ 10000.0, 10000.0 });
    this->declare_parameter(safety_filter_prefix + ".constraints.lsh", std::vector<double>{ 0.0, 0.0 });
    this->declare_parameter(safety_filter_prefix + ".constraints.ush", std::vector<double>{ 10000.0, 10000.0 });

    // Weights
    this->declare_parameter(safety_filter_prefix + ".cost.W_0", std::vector<double>{ 15.0, 3.0, 3.0, 1.0 });
    this->declare_parameter(safety_filter_prefix + ".cost.W", std::vector<double>{ 3.0, 0.5 });

    // Slacks
    double front_Z = 1000.0;
    double rear_Z = 1000.0;
    double front_z = 800.0;
    double rear_z = 800.0;
    this->declare_parameter(safety_filter_prefix + ".cost.Zl", std::vector<double>{ front_Z, rear_Z });
    this->declare_parameter(safety_filter_prefix + ".cost.Zu", std::vector<double>{ front_Z, rear_Z });
    this->declare_parameter(safety_filter_prefix + ".cost.zl", std::vector<double>{ front_z, rear_z });
    this->declare_parameter(safety_filter_prefix + ".cost.zu", std::vector<double>{ front_z, rear_z });

    // Others
    this->declare_parameter(safety_filter_prefix + ".ts", 0.02);
    this->declare_parameter(safety_filter_prefix + ".timeout", 0.05);
}

void MPCSafetyFilterNode::load_parameters() {
    if (!ocp_capsule_) {
        RCLCPP_WARN(this->get_logger(), "load_parameters() called before solver init.");
        return;
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    for (auto & kv : parameter_handlers_) {
        const auto & name = kv.first;
        if (!this->has_parameter(name)) continue;
        auto param = this->get_parameter(name);
        kv.second(param, res);
        if (!res.successful) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to apply initial parameter '%s': %s",
                name.c_str(), res.reason.c_str());
            // reset flag for next parameter
            res.successful = true;
            res.reason.clear();
        }
    }
}

rcl_interfaces::msg::SetParametersResult MPCSafetyFilterNode::on_parameter_update(
    const std::vector<rclcpp::Parameter>& params
) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : params) {
        auto& param_name = param.get_name();

        if (parameter_handlers_.count(param_name)) {
            parameter_handlers_.at(param_name)(param, result);
        } else {
            result.reason = "Update for unknown parameter '" + param_name + "' received.";
            result.successful = false;
        }
    }
    return result;
}

template<size_t N>
void MPCSafetyFilterNode::update_constraint(
    const rclcpp::Parameter& param,
    rcl_interfaces::msg::SetParametersResult& result,
    const char* field,
    const std::vector<int>& stages
) {
    auto values = param.as_double_array();
    bool is_h = strchr(field, 'h') != nullptr;

    size_t expected_size = is_h ? 2 : N;
    if (values.size() != expected_size) {
        result.successful = false;
        result.reason = "Constraint '" + std::string(param.get_name()) + "' has size " +
                      std::to_string(values.size()) + ", but expected is " + std::to_string(expected_size) + ".";
        return;
    }
    
    std::array<double, N> vec{};
    if (is_h) {
        std::array<double, 2> vec_2{ values[0], values[1] };
        vec = create_alternating_array<double, N>(vec_2);
    } else {
        std::copy_n(values.begin(), N, vec.begin());
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Update constraint field '" << field << "' values = " << vec);
    
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    for (int stage : stages) {
        int status = ocp_nlp_constraints_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, ocp_nlp_out_, stage, field, vec.data());
        
        if (status != ACADOS_SUCCESS) {
            result.successful = false;
            result.reason = "Acados solver failed to set cost field '" + std::string(field) + 
                          "' for stage " + std::to_string(stage) + " (error code: " + std::to_string(status) + ")";
            return;
        }
    }
}

template<size_t N>
void MPCSafetyFilterNode::update_cost(
    const rclcpp::Parameter& param,
    rcl_interfaces::msg::SetParametersResult& result,
    const char* field,
    const std::vector<int>& stages
) {
    auto values = param.as_double_array();
    bool is_slack = *field == 'z' || *field == 'Z';
    
    size_t expected_size = is_slack ? 2 : N;
    if (values.size() != expected_size) {
        result.successful = false;
        result.reason = "Cost '" + std::string(param.get_name()) + "' has size " +
                      std::to_string(values.size()) + ", but expected is " + std::to_string(expected_size) + ".";
        return;
    }

    std::array<double, N> vec{};
    std::array<double, N * N> mat{};
    double* data_ptr = nullptr;

    const bool is_weight = strcmp(field, "W") == 0;

    if (is_weight) {
        std::copy_n(values.begin(), N, vec.begin());
        mat = diag_from_vec(vec);
        data_ptr = mat.data();
        RCLCPP_INFO_STREAM(this->get_logger(), "update cost field '" << field << "' mat(flat) = " << mat);
    } 
    else if (is_slack) {
        std::array<double, 2> vec_2{ values[0], values[1] };
        vec = create_alternating_array<double, N>(vec_2);
        data_ptr = vec.data();
        RCLCPP_INFO_STREAM(this->get_logger(), "update cost field '" << field << "' values = " << vec);
    }
    else {
        result.successful = false;
        result.reason = "Not a cost field";
        return;
    }
    
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    for (int stage : stages) {
        int status = ocp_nlp_cost_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, stage, field, data_ptr);
        
        if (status != ACADOS_SUCCESS) {
            result.successful = false;
            result.reason = "Acados solver failed to set cost field '" + std::string(field) + 
                          "' for stage " + std::to_string(stage) + " (error code: " + std::to_string(status) + ")";
            return;
        }
    }
}


// --- ROS Visualizer ---
void MPCSafetyFilterNode::visualize_parameters(const std::vector<Eigen::Vector2d>& obstacles) {
    std::vector<geometry_msgs::msg::Point> geopoints;
    for (const auto& obstacle_vec : obstacles) {
        geometry_msgs::msg::Point point;
        point.x = obstacle_vec.x();
        point.y = obstacle_vec.y();
        point.z = 0.0;
        geopoints.push_back(point);
    }

    Color red_color = {0.6, 0.2, 0.4};
    publish_marker_points(
        geopoints,
        obstacle_marker_pub_,
        "map",
        this->get_clock(),
        red_color,
        "detected_obstacles",
        0,
        std::sqrt(squared_safety_radius_.load()),
        0.8,
        visualization_msgs::msg::Marker::SPHERE_LIST
    );
}

void MPCSafetyFilterNode::visualize_trajectory() {
    std::vector<geometry_msgs::msg::Point> trajectory_points;
    trajectory_points.reserve(AGILEX_N + 1);

    for (int i = 0; i <= AGILEX_N; ++i) {
        double state_at_stage[AGILEX_NX];
        this->get_state(state_at_stage, i);
        geometry_msgs::msg::Point point;
        point.x = state_at_stage[0];
        point.y = state_at_stage[1];
        point.z = 0.0;

        trajectory_points.push_back(point);
    }

    if (!trajectory_points.empty()) {
        Color green_color = {0.0, 1.0, 0.0};

        publish_marker_points(
            trajectory_points,
            trajectory_marker_pub_,
            "map",
            this->get_clock(),
            green_color,
            "predicted_trajectory",
            0,
            0.05,
            1.0,
            visualization_msgs::msg::Marker::LINE_STRIP
        );
    }
}


// --- Helper & Utility Methods ---
void MPCSafetyFilterNode::start_control_timer(double period_seconds) {
    if (control_timer_) control_timer_->cancel();
    if (period_seconds <= 0.0) period_seconds = 0.02;

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_seconds));
    control_timer_ = this->create_wall_timer(
        period,
        std::bind(&MPCSafetyFilterNode::control_loop, this),
        timer_cbg_);
}

void MPCSafetyFilterNode::solver_status_behaviour(const int status, const double unsafe_vel) {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    if (status == ACADOS_SUCCESS) {
        double u0[AGILEX_NU];
        this->get_input(u0, 0);
        
        // Clamp "safe" velocity by unsafe vel
        // tends to do unexpected things in rare cases (maybe memory bug)
        if (std::abs(unsafe_vel) <= EPSILON) {
            u0[0] = 0.0;
        } else if (unsafe_vel > 0.0) {
            u0[0] = std::clamp(u0[0], 0.0, unsafe_vel);
        } else {
            u0[0] = std::clamp(u0[0], unsafe_vel, 0.0);
        }

        this->publish_input(u0);
        first_solve_ = false;
        this->prepare_rti_solve();
    } else {
        this->publish_input(u0_default_.data());
        agilex_acados_reset(ocp_capsule_, 1);
        first_solve_ = true;
    }
}

void MPCSafetyFilterNode::calc_squared_safety_radius() {
    double safety_radius = (last_map_resolution_.load() * SQRT2 + config_.collisions.radius);
    squared_safety_radius_.store(safety_radius * safety_radius);
}

std::array<double, AGILEX_NP> MPCSafetyFilterNode::get_p_values(const std::vector<Eigen::Vector2d> &obstacles) {
    std::array<double, AGILEX_NP> p; 
    p.fill(0.0);
    p[0] = config_.collisions.front_offset;
    p[1] = config_.collisions.back_offset;

    const size_t num_obs_to_set = std::min(obstacles.size(), static_cast<size_t>(MAX_NUM_OBSTACLES));
    for (size_t i = 0; i < num_obs_to_set; ++i) {
        size_t base_idx = 2 + i * 2;
        p[base_idx]     = obstacles[i].x();
        p[base_idx + 1] = obstacles[i].y();
    }

    return p;
}

std::array<double, AGILEX_NH> MPCSafetyFilterNode::get_lh_values(const std::vector<Eigen::Vector2d> &obstacles) {
    std::array<double, AGILEX_NH> h;
    h.fill(-1000.0);

    const size_t num_constraints_to_set = std::min(obstacles.size() * 2, static_cast<size_t>(AGILEX_NH));
    last_num_obstacles_.store(num_constraints_to_set / 2);
    if (num_constraints_to_set > 0) {
        std::fill_n(h.begin(), num_constraints_to_set, squared_safety_radius_.load());
    }
    return h;
}


// --- acados getter --- 
void MPCSafetyFilterNode::get_input(double* u, int stage) {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    ocp_nlp_out_get(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_out_, stage, "u", u);
}

void MPCSafetyFilterNode::get_state(double* x, int stage) {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    ocp_nlp_out_get(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_out_, stage, "x", x);
}


// --- acados setter ---
void MPCSafetyFilterNode::set_x0(double* x0) {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    ocp_nlp_constraints_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, ocp_nlp_out_, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, ocp_nlp_out_, 0, "ubx", x0);
}

void MPCSafetyFilterNode::set_yref0(double* yref0) {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    ocp_nlp_cost_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, 0, "yref", yref0);
}

void MPCSafetyFilterNode::set_lh(double* lh) {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    for (int i = 1; i < AGILEX_N; i++) {
        ocp_nlp_constraints_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, ocp_nlp_out_, i, "lh", lh);
    }
}

void MPCSafetyFilterNode::set_ocp_parameters(double* p, size_t np) {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    for (int i = 0; i <= AGILEX_N; i++) {
        agilex_acados_update_params(ocp_capsule_, i, p, np);
    }
}

void MPCSafetyFilterNode::set_control_bound(double velocity) {
    const double new_lbu0 = std::clamp(velocity, config_.constraints.lbu[0], 0.0);
    const double new_ubu0 = std::clamp(velocity, 0.0, config_.constraints.ubu[0]);

    const bool same = std::abs(new_lbu0 - last_dyn_lbu0_) <= EPSILON &&
        std::abs(new_ubu0 - last_dyn_ubu0_) <= EPSILON;
    if (same) return;

    std::array<double, AGILEX_NBU> lbu = { new_lbu0, config_.constraints.lbu[1] };
    std::array<double, AGILEX_NBU> ubu = { new_ubu0, config_.constraints.ubu[1] };

    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    for (int i = 0; i < AGILEX_N; i++) {
        ocp_nlp_constraints_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, ocp_nlp_out_, i, "lbu", lbu.data());
        ocp_nlp_constraints_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, ocp_nlp_out_, i, "ubu", ubu.data());
    }

    last_dyn_lbu0_ = new_lbu0;
    last_dyn_ubu0_ = new_ubu0;
}

void MPCSafetyFilterNode::warmstart_solver_states(double *x0) {
    std::lock_guard<std::recursive_mutex> solver_lock(solver_mutex_);
    for (int i = 1; i <= AGILEX_N; ++i) {
        ocp_nlp_out_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_out_, ocp_nlp_in_, i, "x", x0);
    }
}

std::vector<Eigen::Vector2d> MPCSafetyFilterNode::set_solver_obstacles(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map, double* x0) {
    Eigen::Vector2d robot_position = {x0[0], x0[1]};
    auto nearest_obstacles = safety_filter_mpc::get_surface_boundary_obstacles(map, robot_position, sb_);
    auto p = this->get_p_values(nearest_obstacles);

    if (static_cast<size_t>(last_num_obstacles_.load()) != nearest_obstacles.size()) {
        auto h = this->get_lh_values(nearest_obstacles);
        this->set_lh(h.data());
    }

    this->set_ocp_parameters(p.data(), p.size());
    return nearest_obstacles;
}

} // namespace safety_filter_mpc

// --- Main Funktion ---
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rcutils_logging_set_logger_level("rcl", RCUTILS_LOG_SEVERITY_WARN);
    rcutils_logging_set_logger_level("rclcpp", RCUTILS_LOG_SEVERITY_WARN);

    auto node = std::make_shared<safety_filter_mpc::MPCSafetyFilterNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}