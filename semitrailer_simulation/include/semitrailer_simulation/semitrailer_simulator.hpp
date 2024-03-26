#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "semitrailer_interfaces/msg/input.hpp"
#include "semitrailer_interfaces/msg/state.hpp"
#include "vehicle_model/semitrailer_model.hpp"

namespace semitrailer_simulation
{
class SemitrailerSimulator : public rclcpp::Node
{
public:
  SemitrailerSimulator(const std::string& node_name, const std::string& ns,
                       const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  explicit SemitrailerSimulator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  using SemitrailerState = vehicle_model::SemitrailerModel::State;
  using SemitrailerInput = vehicle_model::SemitrailerModel::Input;

  void inputCallback(const semitrailer_interfaces::msg::Input::UniquePtr msg);

  void timerCallback();

  double tractor_length_ = 0.;
  double hitch_length_ = 0.;
  double trailer_length_ = 0.;
  int sampling_rate_ = 100;
  std::string global_frame_ = "world";
  std::string base_frame_ = "base_link";
  std::string coupler_joint_ = "coupler_joint";
  std::string tractor_steering_joint_ = "tractor_steering_left_joint";

  SemitrailerState state_;

  SemitrailerInput input_;

  std::unique_ptr<vehicle_model::SemitrailerModel> model_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<semitrailer_interfaces::msg::State>::SharedPtr state_pub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

  rclcpp::Subscription<semitrailer_interfaces::msg::Input>::SharedPtr input_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace semitrailer_simulation
