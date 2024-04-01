#pragma once

#include "vehicle_model/semitrailer_model.hpp"

namespace semitrailer_controller
{
class SemitrailerNLMPC
{
public:
  using SemitrailerModel = vehicle_model::SemitrailerModel;
  using SemitrailerState = SemitrailerModel::State;
  using SemitrailerInput = SemitrailerModel::Input;
  using SemitrailerParam = SemitrailerModel::Param;

  SemitrailerNLMPC(const SemitrailerParam& semitrailer_param);

  void setParameter(const std::size_t prediction_horizon, const double sampling_period,
                    const double damping_coefficient, const double difference_spacing);

  void setWeight(const SemitrailerState& state_weight, const SemitrailerInput& input_weight, double dummy_weight);

  void setLimit(double max_coupler_angle, double max_longitudinal_velocity, double max_steering_angle);

  Eigen::VectorXd computeInitialTracedVar(const SemitrailerState& current_state, const SemitrailerState& ref_state,
                                          const SemitrailerInput& initial_guess);

  Eigen::VectorXd computeTracedVarDot(const SemitrailerState& current_state, const SemitrailerState& current_state_dot,
                                      const Eigen::VectorXd& current_traced_var,
                                      const Eigen::VectorXd& current_traced_var_dot, const SemitrailerState& ref_state);

  std::vector<SemitrailerState> predictTrajectory(const SemitrailerState& current_state,
                                                  const Eigen::VectorXd& current_traced_var);

  SemitrailerInput getRealInput(const Eigen::VectorXd& traced_var);

private:
  static constexpr int NUM_OF_CONSTRAINT = 3;
  static constexpr int NUM_OF_STATE = SemitrailerState::RowsAtCompileTime;
  static constexpr int NUM_OF_REAL_INPUT = SemitrailerInput::RowsAtCompileTime;
  static constexpr int NUM_OF_DUMMY_INPUT = 3;
  static constexpr int NUM_OF_INPUT = NUM_OF_REAL_INPUT + NUM_OF_DUMMY_INPUT;

  using State = Eigen::Matrix<double, NUM_OF_STATE, 1>;
  using Input = Eigen::Matrix<double, NUM_OF_INPUT, 1>;
  using DummyInput = Eigen::Matrix<double, NUM_OF_DUMMY_INPUT, 1>;

  using StateStateJacobian = Eigen::Matrix<double, NUM_OF_STATE, NUM_OF_STATE>;
  using StateInputJacobian = Eigen::Matrix<double, NUM_OF_STATE, NUM_OF_INPUT>;

  using CostStateJacobian = Eigen::Matrix<double, 1, NUM_OF_STATE>;
  using CostInputJacobian = Eigen::Matrix<double, 1, NUM_OF_INPUT>;

  using Constraint = Eigen::Matrix<double, NUM_OF_CONSTRAINT, 1>;
  using ConstraintStateJacobian = Eigen::Matrix<double, NUM_OF_CONSTRAINT, NUM_OF_STATE>;
  using ConstraintInputJacobian = Eigen::Matrix<double, NUM_OF_CONSTRAINT, NUM_OF_INPUT>;

  using HamiltonianStateJacobian = Eigen::Matrix<double, 1, NUM_OF_STATE>;
  using HamiltonianInputJacobian = Eigen::Matrix<double, 1, NUM_OF_INPUT>;

  using StateSeries = Eigen::Matrix<double, NUM_OF_STATE, Eigen::Dynamic>;
  using InputSeries = Eigen::Matrix<double, NUM_OF_INPUT, Eigen::Dynamic>;
  using CostateSeries = Eigen::Matrix<double, NUM_OF_STATE, Eigen::Dynamic>;
  using LagrangeSeries = Eigen::Matrix<double, NUM_OF_CONSTRAINT, Eigen::Dynamic>;
  using TracedVarSeries = Eigen::Matrix<double, NUM_OF_INPUT + NUM_OF_CONSTRAINT, Eigen::Dynamic>;

  double costFunction(const State& state, const Input& input, const State& ref_state);

  CostStateJacobian costStateJacobian(const State& state, const Input& input, const State& ref_state);

  CostInputJacobian costInputJacobian(const State& state, const Input& input, const State& ref_state);

  Constraint constraintFunction(const State& state, const Input& input);

  ConstraintStateJacobian constraintStateJacobian(const State& state, const Input& input);

  ConstraintInputJacobian constraintInputJacobian(const State& state, const Input& input);

  HamiltonianStateJacobian hamiltonianStateJacobian(const State& state, const Input& input, const State& costate,
                                                    const Constraint& lagrange, const State& ref_state);

  HamiltonianInputJacobian hamiltonianInputJacobian(const State& state, const Input& input, const State& costate,
                                                    const Constraint& lagrange, const State& ref_state);

  std::pair<StateSeries, CostateSeries> eulerLagrange(const State& initial_state, const InputSeries& input_series,
                                                      const LagrangeSeries& lagrange_series, const State& ref_state);

  Eigen::VectorXd optimalityFunction(const Eigen::VectorXd& traced_var, const State& state, const State& ref_state);

  StateSeries getStateSeries(const State& initial_state, const InputSeries& input_series);

  SemitrailerModel model_;

  std::size_t prediction_horizon_ = 0;
  double sampling_period_ = 0.;
  double damping_coefficient_ = 0.;
  double difference_spacing_ = 0.;

  SemitrailerState state_weight_ = SemitrailerState::Zero();
  SemitrailerInput input_weight_ = SemitrailerInput::Zero();
  double dummy_weight_ = 0.;

  double max_coupler_angle_ = 0.;
  double max_longitudinal_velocity_ = 0.;
  double max_steering_angle_ = 0.;
};
}  // namespace semitrailer_controller
