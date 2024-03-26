#include "semitrailer_controller/simple_semitrailer_navigation.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace semitrailer_controller
{
SimpleSemitrailerNavigation::SimpleSemitrailerNavigation(const std::string& node_name, const std::string& ns,
                                                         const rclcpp::NodeOptions& options)
  : Node(node_name, ns, options)
{
  using namespace std::placeholders;

  double tractor_length = 0.;
  double hitch_length = 0.;
  double trailer_length = 0.;

  declare_parameter("control_rate", control_rate_);
  declare_parameter("tractor_length", tractor_length);
  declare_parameter("hitch_length", hitch_length);
  declare_parameter("trailer_length", trailer_length);

  get_parameter("control_rate", control_rate_);
  get_parameter("tractor_length", tractor_length);
  get_parameter("hitch_length", hitch_length);
  get_parameter("trailer_length", trailer_length);

  controller_ = std::make_unique<SemitrailerNLMPC>(
      (vehicle_model::SemitrailerModel::Param() << tractor_length, hitch_length, trailer_length).finished());

  input_pub_ = create_publisher<semitrailer_interfaces::msg::Input>("~/input", 10);

  state_sub_ = create_subscription<semitrailer_interfaces::msg::State>(
      "~/state", 10, std::bind(&SimpleSemitrailerNavigation::stateCallback, this, _1));

  simple_navigation_action_server_ = rclcpp_action::create_server<SimpleNavigationAction>(
      this, "~/simple_navigation", std::bind(&SimpleSemitrailerNavigation::handleSimpleNavigationGoal, this, _1, _2),
      std::bind(&SimpleSemitrailerNavigation::handleSimpleNavigationCancel, this, _1),
      std::bind(&SimpleSemitrailerNavigation::handleSimpleNavigationAccepted, this, _1));
}

SimpleSemitrailerNavigation::SimpleSemitrailerNavigation(const rclcpp::NodeOptions& options)
  : SimpleSemitrailerNavigation("semitrailer_nlmpc", "", options)
{
}

void SimpleSemitrailerNavigation::stateCallback(const semitrailer_interfaces::msg::State::UniquePtr msg)
{
  state_ << msg->x, msg->y, msg->theta, msg->beta;
}

rclcpp_action::GoalResponse SimpleSemitrailerNavigation::handleSimpleNavigationGoal(
    const rclcpp_action::GoalUUID& /* uuid */, std::shared_ptr<const SimpleNavigationAction::Goal> /* goal */)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SimpleSemitrailerNavigation::handleSimpleNavigationCancel(
    const std::shared_ptr<SimpleNavigationGoalHandle> /* goal_handle */)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SimpleSemitrailerNavigation::handleSimpleNavigationAccepted(
    const std::shared_ptr<SimpleNavigationGoalHandle> goal_handle)
{
  std::thread(std::bind(&SimpleSemitrailerNavigation::executeSimpleNavigation, this, goal_handle)).detach();
}

void SimpleSemitrailerNavigation::executeSimpleNavigation(const std::shared_ptr<SimpleNavigationGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<SimpleNavigationAction::Feedback>();
  auto result = std::make_shared<SimpleNavigationAction::Result>();

  rclcpp::Rate rate(control_rate_);

  while (rclcpp::ok())
  {
    if (goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
      return;
    }

    rate.sleep();
  }

  goal_handle->succeed(result);
}
};  // namespace semitrailer_controller

RCLCPP_COMPONENTS_REGISTER_NODE(semitrailer_controller::SimpleSemitrailerNavigation)