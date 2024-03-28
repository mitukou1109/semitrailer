#pragma once

#include <unsupported/Eigen/NonLinearOptimization>

namespace semitrailer_controller
{
class NonLinearFunctor
{
public:
  using InputType = Eigen::HybridNonLinearSolver<NonLinearFunctor>::FVectorType;
  using ValueType = Eigen::HybridNonLinearSolver<NonLinearFunctor>::FVectorType;
  using Function = std::function<void(const InputType&, ValueType&)>;

  NonLinearFunctor(Function&& function) : function_(std::forward<Function>(function))
  {
  }

  int operator()(const InputType& input, ValueType& value) const
  {
    function_(input, value);
    return 0;
  }

private:
  Function function_;
};
}  // namespace semitrailer_controller
