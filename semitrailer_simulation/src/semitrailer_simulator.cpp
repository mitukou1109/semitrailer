#include "semitrailer_simulation/semitrailer_simulator.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace semitrailer_simulation
{
SemitrailerSimulator::SemitrailerSimulator(const std::string& node_name, const std::string& ns,
                                           const rclcpp::NodeOptions& options)
  : Node(node_name, ns, options)
{
  std::vector<double> initial_state(4, 0.);

  declare_parameter("tractor_length", tractor_length_);
  declare_parameter("hitch_length", hitch_length_);
  declare_parameter("trailer_length", trailer_length_);
  declare_parameter("sampling_rate", sampling_rate_);
  declare_parameter("global_frame", global_frame_);
  declare_parameter("base_frame", base_frame_);
  declare_parameter("coupler_joint", coupler_joint_);
  declare_parameter("tractor_steering_joint", tractor_steering_joint_);
  declare_parameter("initial_state", initial_state);

  get_parameter("tractor_length", tractor_length_);
  get_parameter("hitch_length", hitch_length_);
  get_parameter("trailer_length", trailer_length_);
  get_parameter("sampling_rate", sampling_rate_);
  get_parameter("global_frame", global_frame_);
  get_parameter("base_frame", base_frame_);
  get_parameter("coupler_joint", coupler_joint_);
  get_parameter("tractor_steering_joint", tractor_steering_joint_);
  get_parameter("initial_state", initial_state);

  state_ = Eigen::Map<SemitrailerState>(initial_state.data(), SemitrailerState::RowsAtCompileTime);

  model_ = std::make_unique<vehicle_model::SemitrailerModel>(tractor_length_, hitch_length_, trailer_length_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  state_pub_ = create_publisher<semitrailer_interfaces::msg::State>("~/state", 10);

  joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);

  input_sub_ = create_subscription<semitrailer_interfaces::msg::Input>(
      "~/input", 10, std::bind(&SemitrailerSimulator::inputCallback, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::milliseconds(std::lround(1000. / sampling_rate_)),
                             std::bind(&SemitrailerSimulator::timerCallback, this));
}

SemitrailerSimulator::SemitrailerSimulator(const rclcpp::NodeOptions& options)
  : SemitrailerSimulator("semitrailer_simulator", "", options)
{
}

void SemitrailerSimulator::inputCallback(const semitrailer_interfaces::msg::Input::UniquePtr msg)
{
  input_ << msg->v, msg->alpha;
}

void SemitrailerSimulator::timerCallback()
{
  state_ += model_->stateFunction(state_, input_) / sampling_rate_;

  semitrailer_interfaces::msg::State state_msg;
  state_msg.x = state_(vehicle_model::SemitrailerModel::X);
  state_msg.y = state_(vehicle_model::SemitrailerModel::Y);
  state_msg.theta = state_(vehicle_model::SemitrailerModel::THETA);
  state_msg.beta = state_(vehicle_model::SemitrailerModel::BETA);
  state_pub_->publish(state_msg);

  geometry_msgs::msg::TransformStamped global_to_base_tf;
  global_to_base_tf.header.stamp = get_clock()->now();
  global_to_base_tf.header.frame_id = global_frame_;
  global_to_base_tf.child_frame_id = base_frame_;
  global_to_base_tf.transform.translation.x = state_(vehicle_model::SemitrailerModel::X);
  global_to_base_tf.transform.translation.y = state_(vehicle_model::SemitrailerModel::Y);
  global_to_base_tf.transform.rotation = tf2::toMsg({ { 0, 0, 1 }, state_(vehicle_model::SemitrailerModel::THETA) });
  tf_broadcaster_->sendTransform(global_to_base_tf);

  sensor_msgs::msg::JointState joint_states_msg;
  joint_states_msg.header.stamp = get_clock()->now();
  joint_states_msg.name = { coupler_joint_, tractor_steering_joint_ };
  joint_states_msg.position = { state_(vehicle_model::SemitrailerModel::BETA),
                                input_(vehicle_model::SemitrailerModel::ALPHA) };
  joint_states_pub_->publish(joint_states_msg);
}
}  // namespace semitrailer_simulation

RCLCPP_COMPONENTS_REGISTER_NODE(semitrailer_simulation::SemitrailerSimulator)
