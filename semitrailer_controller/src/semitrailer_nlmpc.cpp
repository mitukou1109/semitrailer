#include "semitrailer_controller/semitrailer_nlmpc.hpp"

#include "semitrailer_controller/non_linear_functor.hpp"

namespace semitrailer_controller
{
SemitrailerNLMPC::SemitrailerNLMPC(const SemitrailerParam& semitrailer_param) : model_(semitrailer_param)
{
}

void SemitrailerNLMPC::setParameter(const std::size_t prediction_horizon, const double sampling_period,
                                    const double damping_coefficient, const double difference_spacing)
{
  prediction_horizon_ = prediction_horizon;
  sampling_period_ = sampling_period;
  damping_coefficient_ = damping_coefficient;
  difference_spacing_ = difference_spacing;

  initialized_ = false;
}

void SemitrailerNLMPC::setWeight(const SemitrailerState& state_weight, const SemitrailerInput& input_weight,
                                 double dummy_weight)
{
  state_weight_ = state_weight;
  input_weight_ = input_weight;
  dummy_weight_ = dummy_weight;
}

void SemitrailerNLMPC::setLimit(double max_coupler_angle, double max_longitudinal_velocity, double max_steering_angle)
{
  max_coupler_angle_ = max_coupler_angle;
  max_longitudinal_velocity_ = max_longitudinal_velocity;
  max_steering_angle_ = max_steering_angle;
}

SemitrailerNLMPC::Result SemitrailerNLMPC::initialize(const SemitrailerState& current_state,
                                                      const SemitrailerState& ref_state,
                                                      const SemitrailerInput& initial_guess)
{
  if (std::abs(current_state(SemitrailerModel::BETA)) > max_coupler_angle_ ||
      std::abs(initial_guess(SemitrailerModel::V)) > max_longitudinal_velocity_ ||
      std::abs(initial_guess(SemitrailerModel::ALPHA)) > max_steering_angle_)
  {
    return {};
  }

  DummyInput initial_dummy;
  initial_dummy << max_coupler_angle_ - std::abs(current_state(SemitrailerModel::BETA)),
      max_longitudinal_velocity_ - std::abs(current_state(SemitrailerModel::V)),
      max_steering_angle_ - std::abs(current_state(SemitrailerModel::ALPHA));

  traced_var_vec_ = (Eigen::VectorXd() << initial_guess, initial_dummy, Constraint::Zero())
                        .finished()
                        .replicate(prediction_horizon_, 1);

  NonLinearFunctor functor([this, current_state, ref_state](const NonLinearFunctor::InputType& traced_var_vec,
                                                            NonLinearFunctor::ValueType& residual) {
    residual = optimalityFunction(traced_var_vec, current_state, ref_state);
  });

  Eigen::HybridNonLinearSolver solver(functor);
  solver.solveNumericalDiff(traced_var_vec_);

  traced_var_dot_vec_ = Eigen::VectorXd::Zero((NUM_OF_INPUT + NUM_OF_CONSTRAINT) * prediction_horizon_);

  initialized_ = true;

  return generateResult(current_state);
}

SemitrailerNLMPC::Result SemitrailerNLMPC::computeInput(const SemitrailerState& current_state,
                                                        const SemitrailerState& ref_state)
{
  if (!initialized_)
  {
    return {};
  }

  NonLinearFunctor functor([this, current_state, ref_state](const NonLinearFunctor::InputType& traced_var_dot_vec,
                                                            NonLinearFunctor::ValueType& residual) {
    const auto real_input = traced_var_vec_.head(NUM_OF_REAL_INPUT);
    auto F_t = optimalityFunction(traced_var_vec_, current_state, ref_state);

    residual = (optimalityFunction(
                    traced_var_vec_ + difference_spacing_ * traced_var_dot_vec,
                    current_state + difference_spacing_ * model_.stateFunction(current_state, real_input), ref_state) -
                F_t) /
                   difference_spacing_ +
               damping_coefficient_ * F_t;
  });

  Eigen::HybridNonLinearSolver solver(functor);
  solver.solveNumericalDiff(traced_var_dot_vec_);

  traced_var_vec_ += sampling_period_ * traced_var_dot_vec_;

  return generateResult(current_state);
}

double SemitrailerNLMPC::costFunction(const State& state, const Input& input, const State& ref_state)
{
  const auto state_error = state - ref_state;
  const auto real_input = input.head(NUM_OF_REAL_INPUT);
  const auto dummy_input = input.tail(NUM_OF_DUMMY_INPUT);

  double L = 0.;
  L += state_error.transpose() * state_weight_.asDiagonal() * state_error;
  L += real_input.transpose() * input_weight_.asDiagonal() * real_input;
  L += -dummy_weight_ * dummy_input.sum();

  return L;
}

SemitrailerNLMPC::CostStateJacobian SemitrailerNLMPC::costStateJacobian(const State& state, const Input& /* input */,
                                                                        const State& ref_state)
{
  const auto state_error = state - ref_state;

  CostStateJacobian dLdx = 2 * state_weight_.array() * state_error.array();

  return dLdx;
}

SemitrailerNLMPC::CostInputJacobian SemitrailerNLMPC::costInputJacobian(const State& /* state */, const Input& input,
                                                                        const State& /* ref_state */)
{
  const auto real_input = input.head(NUM_OF_REAL_INPUT);

  CostInputJacobian dLdu = CostInputJacobian::Zero();
  dLdu.leftCols(NUM_OF_REAL_INPUT) = 2 * input_weight_.array() * real_input.array();
  dLdu.rightCols(NUM_OF_DUMMY_INPUT) = -dummy_weight_ * DummyInput::Ones();

  return dLdu;
}

SemitrailerNLMPC::Constraint SemitrailerNLMPC::constraintFunction(const State& state, const Input& input)
{
  const auto real_input = input.head(NUM_OF_REAL_INPUT);
  const auto dummy_input = input.tail(NUM_OF_DUMMY_INPUT);

  Constraint C;
  C << std::pow(state(SemitrailerModel::BETA), 2) + std::pow(dummy_input(0), 2) - std::pow(max_coupler_angle_, 2),
      std::pow(real_input(SemitrailerModel::V), 2) + std::pow(dummy_input(1), 2) -
          std::pow(max_longitudinal_velocity_, 2),
      std::pow(real_input(SemitrailerModel::ALPHA), 2) + std::pow(dummy_input(2), 2) - std::pow(max_steering_angle_, 2);

  return C;
}

SemitrailerNLMPC::ConstraintStateJacobian SemitrailerNLMPC::constraintStateJacobian(const State& state,
                                                                                    const Input& /* input */)
{
  ConstraintStateJacobian dCdx;
  dCdx(0, SemitrailerModel::BETA) = 2 * state(SemitrailerModel::BETA);

  return dCdx;
}

SemitrailerNLMPC::ConstraintInputJacobian SemitrailerNLMPC::constraintInputJacobian(const State& /* state */,
                                                                                    const Input& input)
{
  const auto dummy_input = input.tail(NUM_OF_DUMMY_INPUT);

  ConstraintInputJacobian dCdu;
  dCdu(1, SemitrailerModel::V) = 2 * input(SemitrailerModel::V);
  dCdu(2, SemitrailerModel::ALPHA) = 2 * input(SemitrailerModel::ALPHA);
  dCdu.rightCols(NUM_OF_DUMMY_INPUT) = 2 * dummy_input.asDiagonal();

  return dCdu;
}

SemitrailerNLMPC::HamiltonianStateJacobian SemitrailerNLMPC::hamiltonianStateJacobian(
    const State& state, const Input& input, const State& costate, const Constraint& lagrange, const State& ref_state)
{
  const auto real_input = input.head(NUM_OF_REAL_INPUT);

  HamiltonianStateJacobian dHdx = costStateJacobian(state, input, ref_state) +
                                  costate.transpose() * model_.stateJacobian(state, real_input) +
                                  lagrange.transpose() * constraintStateJacobian(state, input);

  return dHdx;
}

SemitrailerNLMPC::HamiltonianInputJacobian SemitrailerNLMPC::hamiltonianInputJacobian(
    const State& state, const Input& input, const State& costate, const Constraint& lagrange, const State& ref_state)
{
  const auto real_input = input.head(NUM_OF_REAL_INPUT);

  StateInputJacobian state_input_jacobian = StateInputJacobian::Zero();
  state_input_jacobian.leftCols(NUM_OF_REAL_INPUT) = model_.inputJacobian(state, real_input);

  HamiltonianInputJacobian dHdu = costInputJacobian(state, input, ref_state) +
                                  costate.transpose() * state_input_jacobian +
                                  lagrange.transpose() * constraintInputJacobian(state, input);

  return dHdu;
}

SemitrailerNLMPC::StateSeries SemitrailerNLMPC::predictTrajectory(const State& initial_state,
                                                                  const InputSeries& input_series)
{
  StateSeries state_series(NUM_OF_STATE, prediction_horizon_ + 1);
  state_series.col(0) = initial_state;

  for (std::size_t i = 0; i < prediction_horizon_; ++i)
  {
    state_series.col(i + 1) =
        state_series.col(i) +
        model_.stateFunction(state_series.col(i), input_series.col(i).head(NUM_OF_REAL_INPUT)) * sampling_period_;
  }

  return state_series;
}

std::pair<SemitrailerNLMPC::StateSeries, SemitrailerNLMPC::CostateSeries>
SemitrailerNLMPC::eulerLagrange(const State& initial_state, const InputSeries& input_series,
                                const LagrangeSeries& lagrange_series, const State& ref_state)
{
  auto state_series = predictTrajectory(initial_state, input_series);

  CostateSeries costate_series(NUM_OF_STATE, prediction_horizon_);

  costate_series.col(prediction_horizon_ - 1) =
      costStateJacobian(state_series.col(prediction_horizon_), Input::Zero(), ref_state).transpose();

  for (std::size_t i = prediction_horizon_ - 1; i > 0; --i)
  {
    costate_series.col(i - 1) =
        costate_series.col(i) + hamiltonianStateJacobian(state_series.col(i), input_series.col(i),
                                                         costate_series.col(i), lagrange_series.col(i), ref_state)
                                        .transpose() *
                                    sampling_period_;
  }

  return { state_series, costate_series };
}

Eigen::VectorXd SemitrailerNLMPC::optimalityFunction(const Eigen::VectorXd& traced_var_vec, const State& state,
                                                     const State& ref_state)
{
  const Eigen::Map<const TracedVarSeries> traced_var_series(traced_var_vec.data(), NUM_OF_INPUT + NUM_OF_CONSTRAINT,
                                                            prediction_horizon_);
  const auto input_series = traced_var_series.topRows(NUM_OF_INPUT);
  const auto lagrange_series = traced_var_series.bottomRows(NUM_OF_CONSTRAINT);

  const auto [state_series, costate_series] = eulerLagrange(state, input_series, lagrange_series, ref_state);

  Eigen::MatrixXd F(NUM_OF_INPUT + NUM_OF_CONSTRAINT, prediction_horizon_);
  for (std::size_t i = 0; i < prediction_horizon_; ++i)
  {
    F.col(i) << hamiltonianInputJacobian(state_series.col(i), input_series.col(i), costate_series.col(i),
                                         lagrange_series.col(i), ref_state)
                    .transpose(),
        constraintFunction(state_series.col(i), input_series.col(i));
  }

  return Eigen::Map<Eigen::VectorXd>(F.data(), F.size());
}

SemitrailerNLMPC::Result SemitrailerNLMPC::generateResult(const SemitrailerState& current_state)
{
  const Eigen::Map<const TracedVarSeries> traced_var_series(traced_var_vec_.data(), NUM_OF_INPUT + NUM_OF_CONSTRAINT,
                                                            prediction_horizon_);
  const auto input_series = traced_var_series.topRows(NUM_OF_INPUT);

  Result result;

  result.input = input_series.col(0).head(NUM_OF_REAL_INPUT);

  auto predicted_trajectory = predictTrajectory(current_state, input_series);
  for (std::size_t i = 0; i < prediction_horizon_; ++i)
  {
    result.predicted_trajectory.push_back(predicted_trajectory.col(i));
  }

  return result;
}
}  // namespace semitrailer_controller
