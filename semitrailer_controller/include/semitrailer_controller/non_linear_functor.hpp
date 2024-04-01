#pragma once

#include <unsupported/Eigen/NonLinearOptimization>

namespace semitrailer_controller
{
class NonLinearFunctor
{
public:
  using Function = std::function<bool(double const* const*, double*)>;

  NonLinearFunctor(Function&& function) : function_(std::forward<Function>(function))
  {
  }

  bool operator()(double const* const* parameters, double* residuals) const
  {
    return function_(parameters, residuals);
  }

private:
  Function function_;
};
}  // namespace semitrailer_controller
