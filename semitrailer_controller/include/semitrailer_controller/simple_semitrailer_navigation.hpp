#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "semitrailer_controller/semitrailer_nlmpc.hpp"
#include "semitrailer_interfaces/action/simple_navigation.hpp"
#include "semitrailer_interfaces/msg/input.hpp"
#include "semitrailer_interfaces/msg/state.hpp"

namespace semitrailer_controller
{

class SimpleSemitrailerNavigation : public rclcpp::Node
{
public:
  SimpleSemitrailerNavigation(const std::string& node_name, const std::string& ns,
                              const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  SimpleSemitrailerNavigation(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  using SimpleNavigationAction = semitrailer_interfaces::action::SimpleNavigation;
  using SimpleNavigationGoalHandle = rclcpp_action::ServerGoalHandle<SimpleNavigationAction>;
  using SemitrailerState = vehicle_model::SemitrailerModel::State;
  using SemitrailerInput = vehicle_model::SemitrailerModel::Input;

  void stateCallback(const semitrailer_interfaces::msg::State::UniquePtr msg);

  rclcpp_action::GoalResponse handleSimpleNavigationGoal(const rclcpp_action::GoalUUID& uuid,
                                                         std::shared_ptr<const SimpleNavigationAction::Goal> goal);

  rclcpp_action::CancelResponse
  handleSimpleNavigationCancel(const std::shared_ptr<SimpleNavigationGoalHandle> goal_handle);

  void handleSimpleNavigationAccepted(const std::shared_ptr<SimpleNavigationGoalHandle> goal_handle);

  void executeSimpleNavigation(const std::shared_ptr<SimpleNavigationGoalHandle> goal_handle);

  int control_rate_ = 10;

  std::unique_ptr<SemitrailerNLMPC> controller_;

  rclcpp::Subscription<semitrailer_interfaces::msg::State>::SharedPtr state_sub_;

  rclcpp::Publisher<semitrailer_interfaces::msg::Input>::SharedPtr input_pub_;

  rclcpp_action::Server<SimpleNavigationAction>::SharedPtr simple_navigation_action_server_;

  SemitrailerState state_;

  SemitrailerInput input_;
};

}  // namespace semitrailer_controller
