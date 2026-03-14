#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <offboard_rl/utils.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct waypoint
{
    double x, y, z, yaw;
    double duration; // Time to reach this waypoint from previous one
};

class TrajectoryPlanner : public rclcpp::Node
{
public:
    TrajectoryPlanner() : Node("trajectory_planner")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
            qos, std::bind(&TrajectoryPlanner::vehicle_local_position_callback, this, std::placeholders::_1));
        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude",
            qos, std::bind(&TrajectoryPlanner::vehicle_attitude_callback, this, std::placeholders::_1));
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        timer_offboard_ = this->create_wall_timer(100ms, std::bind(&TrajectoryPlanner::activate_offboard, this));
        timer_trajectory_publish_ = this->create_wall_timer(20ms, std::bind(&TrajectoryPlanner::publish_trajectory_setpoint, this));

        // Define trajectory waypoints (7 waypoints + start)
        // Circular trajectory with altitude variation
        waypoints_ = {
            {  0.0,   0.0,  -3.0,    0.0,  4.0 },   // WP1: takeoff to 3 m
            { 10.0,   0.0,  -5.0,    0.0,  5.0 },   // WP2: climb to 5 m while moving north
            { 10.0,  10.0,  -6.0,   45.0,  5.0 },   // WP3: move east, climb to 6 m, yaw 45°
            {  0.0,  10.0,  -4.0,   90.0,  5.0 },   // WP4: move south, descend to 4 m
            { -5.0,   5.0,  -7.0,  135.0,  6.0 },   // WP5: diagonal, climb to highest point (7 m)
            {  0.0,   0.0,  -5.0,  180.0,  6.0 },   // WP6: return toward start, descend to 5 m
            {  0.0,   0.0,   0.0,    0.0,  5.0 }    // WP7: land
        };

        std::cout << "Trajectory planner initialized with " << waypoints_.size() << " waypoints" << std::endl;
        std::cout << "Waiting for offboard activation..." << std::endl;
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    rclcpp::TimerBase::SharedPtr timer_offboard_;
    rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;

    std::vector<waypoint> waypoints_;
    size_t current_waypoint{0};
    bool offboard_active_{false};
    bool trajectory_started_{false};
    bool trajectory_completed_{false};
    double segment_time_{0.0};
    double offboard_counter_{0};

    Eigen::Vector<double, 6> trajectory_coeffs_;
    Eigen::Vector4d segment_start_, segment_end_;
    double segment_duration_;
    double sigma{0.5}; // Small time margin to start the next waypoint before it ends the current one


    VehicleLocalPosition current_position_{};
    VehicleAttitude current_attitude_{};

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_position_ = *msg;
    }

    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        current_attitude_ = *msg;
    }

    void activate_offboard()
    {
        if(offboard_counter_ == 10) {
            // Change to Offboard mode
            VehicleCommand msg{};
            msg.param1 = 1;
            msg.param2 = 6;
            msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(msg);

            // Arm the vehicle
            msg.param1 = 1.0;
            msg.param2 = 0.0;
            msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(msg);

            offboard_active_ = true;
            std::cout << "Offboard mode activated! Starting trajectory..." << std::endl;
        }

        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);

        if (offboard_counter_ < 11) offboard_counter_++;
    }

    void publish_trajectory_setpoint()
    {
        if (!offboard_active_) {
            return;
        }

        // Initialize first segment
        if (!trajectory_started_) {
            segment_start_(0) = current_position_.x;
            segment_start_(1) = current_position_.y;
            segment_start_(2) = current_position_.z;
            auto rpy = utilities::quatToRpy(Vector4d(current_attitude_.q[0], current_attitude_.q[1], 
                                                      current_attitude_.q[2], current_attitude_.q[3]));
            segment_start_(3) = rpy[2];
            
            load_next_waypoint();
            trajectory_started_ = true;
        }

        // Check if current segment is complete
        if (segment_time_ > segment_duration_ - sigma) {
            if (current_waypoint < waypoints_.size()) {
                // Move to next waypoint
                segment_start_ = segment_end_;
                load_next_waypoint();
                segment_time_ = 0.0;
                std::cout << "Moving to waypoint " << current_waypoint << "/" << waypoints_.size() << std::endl;
            } else {
                // Trajectory complete
                if(!trajectory_completed_){
                trajectory_completed_ = true;
                std::cout << "Trajectory completed!" << std::endl;
                }
                return;
            }
        }

        // Compute and publish trajectory setpoint
        TrajectorySetpoint msg = compute_trajectory_setpoint(segment_time_);
        trajectory_setpoint_publisher_->publish(msg);
        
        segment_time_ += 0.02; // 20ms timestep
    }

    void load_next_waypoint()
    {
        if (current_waypoint > waypoints_.size()-1) {
            return;
        }

        const waypoint& next_wp = waypoints_[current_waypoint];
        segment_end_(0) = next_wp.x;
        segment_end_(1) = next_wp.y;
        segment_end_(2) = next_wp.z;
        segment_end_(3) = next_wp.yaw;
        segment_duration_ = next_wp.duration;

        // Compute polynomial coefficients for this segment
        compute_polynomial_coefficients();
        
        current_waypoint++;
    }

    void compute_polynomial_coefficients()
    {
        Eigen::Vector4d e = segment_end_ - segment_start_;
        e(3) = utilities::angleError(segment_end_(3), segment_start_(3));
        double s_f = e.norm();

        Eigen::VectorXd b(6);
        Eigen::Matrix<double, 6, 6> A;

        double T = segment_duration_;
        
        b << 0.0, 0.0, 0.0, s_f, 0.0, 0.0;
        A << 0, 0, 0, 0, 0, 1,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 1, 0, 0,
             pow(T,5), pow(T,4), pow(T,3), pow(T,2), T, 1,
             5*pow(T,4), 4*pow(T,3), 3*pow(T,2), 2*T, 1, 0,
             20*pow(T,3), 12*pow(T,2), 6*T, 2, 0, 0;

        trajectory_coeffs_ = A.inverse() * b;
    }

    TrajectorySetpoint compute_trajectory_setpoint(double t)
    {
        Eigen::Vector4d e = segment_end_ - segment_start_;
        e(3) = utilities::angleError(segment_end_(3), segment_start_(3));
        double s_f = e.norm();

        if (s_f < 1e-6) {
            // Avoid division by zero for very close waypoints
            TrajectorySetpoint msg{};
            msg.position = {float(segment_end_(0)), float(segment_end_(1)), float(segment_end_(2))};
            msg.velocity = {0.0f, 0.0f, 0.0f};
            msg.acceleration = {0.0f, 0.0f, 0.0f};
            msg.yaw = float(segment_end_(3));
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            return msg;
        }

        const auto& x = trajectory_coeffs_;

        double s = x(0) * std::pow(t, 5.0)
                 + x(1) * std::pow(t, 4.0)
                 + x(2) * std::pow(t, 3.0)
                 + x(3) * std::pow(t, 2.0)
                 + x(4) * t
                 + x(5);

        double s_d = 5.0 * x(0) * std::pow(t, 4.0)
                   + 4.0 * x(1) * std::pow(t, 3.0)
                   + 3.0 * x(2) * std::pow(t, 2.0)
                   + 2.0 * x(3) * t
                   + x(4);

        double s_dd = 20.0 * x(0) * std::pow(t, 3.0)
                    + 12.0 * x(1) * std::pow(t, 2.0)
                    +  6.0 * x(2) * t
                    +  2.0 * x(3);

        Eigen::Vector4d ref_pos = segment_start_ + s * e / s_f;
        Eigen::Vector4d ref_vel = s_d * e / s_f;
        Eigen::Vector4d ref_acc = s_dd * e / s_f;

        TrajectorySetpoint msg{};
        msg.position = {float(ref_pos(0)), float(ref_pos(1)), float(ref_pos(2))};
        msg.velocity = {float(ref_vel(0)), float(ref_vel(1)), float(ref_vel(2))};
        msg.acceleration = {float(ref_acc(0)), float(ref_acc(1)), float(ref_acc(2))};
        msg.yaw = float(ref_pos(3));
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        return msg;
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting trajectory planner node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}