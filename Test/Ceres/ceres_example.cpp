#include <iostream>
#include <ceres/ceres.h>
#include <cmath>

struct MyCostFunction
{
  MyCostFunction(double x, double y)
      : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T *const parameters, T *residual) const
  {
    T a = parameters[0];
    T b = parameters[1];
    T omega = parameters[2];
    T theta = parameters[3];

    residual[0] = y_ - a * ceres::sin(omega * x_ + theta) - b;
    return true;
  }

  static ceres::CostFunction *Create(const double x, const double y)
  {
    return new ceres::AutoDiffCostFunction<MyCostFunction, 1, 4>(
        new MyCostFunction(x, y));
  }

private:
  const double x_;
  const double y_;
};

// 回调函数用于打印优化前的参数值
class ParameterCallback : public ceres::IterationCallback
{
public:
  explicit ParameterCallback(double *parameters)
      : parameters_(parameters) {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary)
  {
    std::cout << "Before optimization:" << std::endl;
    std::cout << "a = " << parameters_[0] << std::endl;
    std::cout << "b = " << parameters_[1] << std::endl;
    std::cout << "omega = " << parameters_[2] << std::endl;
    std::cout << "theta = " << parameters_[3] << std::endl;

    return ceres::SOLVER_CONTINUE;
  }

private:
  double *parameters_;
};

int main()
{
  const int num_data_points = 20;
  double x[num_data_points];
  double y[num_data_points];

  for (int i = 0; i < num_data_points; ++i)
  {
    x[i] = i;
    y[i] = 2.0 * std::sin(0.5 * x[i] + 0.1) + 0.5;
  }

  ceres::Problem problem;

  double parameters[4] = {0.5, 0.0, 0.5, 0.0}; // 根据实际情况进行初始化

  for (int i = 0; i < num_data_points; ++i)
  {
    problem.AddResidualBlock(
        MyCostFunction::Create(x[i], y[i]),
        nullptr,
        parameters);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  // 创建回调函数对象并添加到选项中
  ParameterCallback parameter_callback(parameters);
  options.callbacks.push_back(&parameter_callback);

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "Final parameters:" << std::endl;
  std::cout << "a = " << parameters[0] << std::endl;
  std::cout << "b = " << parameters[1] << std::endl;
  std::cout << "omega = " << parameters[2] << std::endl;
  std::cout << "theta = " << parameters[3] << std::endl;

  return 0;
}
