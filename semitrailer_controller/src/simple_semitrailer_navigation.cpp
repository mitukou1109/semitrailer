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
  int prediction_horizon = 0;
  double sampling_period = 0.;
  double damping_coefficient = 0.;
  double difference_spacing = 0.;
  std::vector<double> state_weight(SemitrailerState::SizeAtCompileTime, 0.);
  std::vector<double> input_weight(SemitrailerInput::SizeAtCompileTime, 0.);
  double dummy_weight = 0.;
  double max_coupler_angle = 0.;
  double max_longitudinal_velocity = 0.;
  double max_steering_angle = 0.;

  declare_parameter("control_rate", control_rate_);

  declare_parameter("tractor_length", tractor_length);
  declare_parameter("hitch_length", hitch_length);
  declare_parameter("trailer_length", trailer_length);
  declare_parameter("prediction_horizon", prediction_horizon);
  declare_parameter("sampling_period", sampling_period);
  declare_parameter("damping_coefficient", damping_coefficient);
  declare_parameter("difference_spacing", difference_spacing);
  declare_parameter("state_weight", state_weight);
  declare_parameter("input_weight", input_weight);
  declare_parameter("dummy_weight", dummy_weight);
  declare_parameter("max_coupler_angle", max_coupler_angle);
  declare_parameter("max_longitudinal_velocity", max_longitudinal_velocity);
  declare_parameter("max_steering_angle", max_steering_angle);

  get_parameter("control_rate", control_rate_);

  get_parameter("tractor_length", tractor_length);
  get_parameter("hitch_length", hitch_length);
  get_parameter("trailer_length", trailer_length);
  get_parameter("prediction_horizon", prediction_horizon);
  get_parameter("sampling_period", sampling_period);
  get_parameter("damping_coefficient", damping_coefficient);
  get_parameter("difference_spacing", difference_spacing);
  get_parameter("state_weight", state_weight);
  get_parameter("input_weight", input_weight);
  get_parameter("dummy_weight", dummy_weight);
  get_parameter("max_coupler_angle", max_coupler_angle);
  get_parameter("max_longitudinal_velocity", max_longitudinal_velocity);
  get_parameter("max_steering_angle", max_steering_angle);

  controller_ = std::make_unique<SemitrailerNLMPC>(
      (vehicle_model::SemitrailerModel::Param() << tractor_length, hitch_length, trailer_length).finished());
  controller_->setParameter(prediction_horizon, sampling_period, damping_coefficient, difference_spacing);
  controller_->setWeight(Eigen::Map<SemitrailerState>(state_weight.data(), SemitrailerState::SizeAtCompileTime),
                         Eigen::Map<SemitrailerInput>(input_weight.data(), SemitrailerInput::SizeAtCompileTime),
                         dummy_weight);
  controller_->setLimit(max_coupler_angle, max_longitudinal_velocity, max_steering_angle);

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
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ << msg->x, msg->y, msg->theta, msg->beta;
  }
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

  SemitrailerState state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = state_;
  }
  SemitrailerState state_dot = SemitrailerState::Zero();

  SemitrailerState ref_state;
  ref_state << goal->goal.x, goal->goal.y, goal->goal.theta, goal->goal.beta;
  auto traced_var = controller_->computeInitialTracedVar(state, ref_state, (SemitrailerInput() << 0., 0.).finished());
  Eigen::VectorXd traced_var_dot = Eigen::VectorXd::Zero(traced_var.size());

  publishInput(controller_->getRealInput(traced_var));

  rclcpp::Rate rate(control_rate_);

  while (rclcpp::ok())
  {
    if (!rate.sleep())
    {
      RCLCPP_WARN(get_logger(), "Control loop missed its desired rate of %dHz", control_rate_);
    }

    if (goal_handle->is_canceling())
    {
      RCLCPP_INFO(get_logger(), "Goal canceled");
      goal_handle->canceled(result);
      break;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_dot = (state_ - state) * control_rate_;
      state = state_;
    }

    traced_var_dot = controller_->computeTracedVarDot(state, state_dot, traced_var, traced_var_dot, ref_state);
    traced_var += traced_var_dot / control_rate_;

    publishInput(controller_->getRealInput(traced_var));
  }

  if (goal_handle->is_active())
  {
    goal_handle->succeed(result);
  }

  publishInput(SemitrailerInput::Zero());
}

void SimpleSemitrailerNavigation::publishInput(const SemitrailerInput& input)
{
  semitrailer_interfaces::msg::Input input_msg;
  input_msg.v = input(SemitrailerModel::V);
  input_msg.alpha = input(SemitrailerModel::ALPHA);
  input_pub_->publish(input_msg);
}
};  // namespace semitrailer_controller

RCLCPP_COMPONENTS_REGISTER_NODE(semitrailer_controller::SimpleSemitrailerNavigation)
