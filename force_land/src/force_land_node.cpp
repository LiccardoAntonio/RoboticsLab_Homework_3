#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

using namespace std::chrono_literals;

class ForceLand : public rclcpp::Node
{
	public:
	ForceLand() : Node("force_land"), need_land(false), safety_mode(false)
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
		qos, std::bind(&ForceLand::height_callback, this, std::placeholders::_1));
		publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
		subscription_vld_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
		qos, std::bind(&ForceLand::landed_callback, this, std::placeholders::_1));
		subscription_cm_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>("/fmu/out/vehicle_control_mode",
		qos, std::bind(&ForceLand::control_mode_callback, this, std::placeholders::_1));
		timer_ = this->create_wall_timer(10ms, std::bind(&ForceLand::activate_switch, this));
	}

	private:
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr subscription_vld_;
	rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr subscription_cm_;


	rclcpp::TimerBase::SharedPtr timer_;

	bool need_land;
	bool is_landing;
	bool is_landed;
	bool in_descend;
	bool manual_mode;
	bool auto_mode;
	bool safety_mode;
	bool above_threshold;



	void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) 
	{
		float z_ = -msg->z;
		//std::cout << "Current drone height: " << z_ << " meters" <<  std::endl;
		if(z_ > 20)
		{
			need_land = true;
			above_threshold = true;
		}
		else{
			above_threshold = false;
		}

		return;
	}

	void landed_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
	{
		in_descend = msg->in_descend;
		is_landed = msg->landed;

	}

	void control_mode_callback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg)
	{
		manual_mode = msg->flag_control_manual_enabled;
	}

	void activate_switch()
	{
		if(is_landed)
		{
			is_landing = false;
		}
		if(is_landing && manual_mode && !above_threshold){
			std::cout << "Manual mode detected, cancelling forced landing." << std::endl;
			safety_mode = true;
			is_landing = false;
			need_land = false;
			return;
		}

		if(need_land && !safety_mode)
		{
			std::cout << "Drone height exceeded 20 meters threshold, Landing forced" << std::endl;
			auto command = px4_msgs::msg::VehicleCommand();
			command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
			this->publisher_->publish(command);
			is_landing = true;
			need_land = false;
		}
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_local_position listener node..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ForceLand>());
	rclcpp::shutdown();
	return 0;
}