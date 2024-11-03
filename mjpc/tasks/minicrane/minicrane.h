#ifndef MJPC_TASKS_MINICRANE_MINICRANE_H_
#define MJPC_TASKS_MINICRANE_MINICRANE_H_

#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {
class Minicrane : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  int count = 0;

  class ResidualFn : public BaseResidualFn {
   public:
    explicit ResidualFn(const Minicrane* task) : BaseResidualFn(task) {}
    // ------- Residuals for ccrane task ------
    //   Number of residuals: 3
    //     Residual (0): distance from payload x pos
    //     Residual (1): distance from payload y pos
    //     Residual (2): distance from payload z pos
    // ------------------------------------------
    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;
  };

  Minicrane() : residual_(this) {}
  void TransitionLocked(mjModel* model, mjData* data) override;

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;
};
}  // namespace mjpc

#endif  // MJPC_TASKS_CRANE_CRANE_H_
