#include "mjpc/tasks/minicrane/minicrane.h"

#include <cmath>
#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"
#include <absl/random/random.h>
#include <mujoco/mjtnum.h>


namespace mjpc {
std::string Minicrane::XmlPath() const {
  return GetModelPath("minicrane/task.xml");
}
std::string Minicrane::Name() const { return "Minicrane"; }

void Minicrane::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                    double* residual) const {
  int counter = 0;
  // ---------- Residual (0) ----------
  // ---------- Cart Distance to Target ----
  double* payload_pos = SensorByName(model, data, "p_pos");
  mjtNum dif[3] = {parameters_[0]-payload_pos[0], parameters_[1]-payload_pos[1], parameters_[2]-payload_pos[2]};
  residual[0] = mju_sqrt(dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2]);
  counter++;

  // ---------- Residual (1) ----------
  // ---------- Control ----------
  residual[1] = data->ctrl[0];
  residual[2] = data->ctrl[1];
  residual[3] = data->ctrl[2];
  counter += 3;

  // ---------- Residual (2) ----------
  // ---------- Orientation of Payload ----------
  double* p_quat = SensorByName(model, data, "p_quat");
  mjtNum normal_quat[4] = {0, 0, 0, 1};
  mju_normalize4(p_quat);
  mju_subQuat(residual + counter, normal_quat, p_quat);
  counter += 4;

  // ---------- Residual (3) ----------
  // ---------- Angular Acceleration of Payload ----------
  double* p_acc = SensorByName(model, data, "angle_acc");
  mju_normalize3(p_acc);
  mju_copy3(residual + counter, p_acc);
  counter += 3;

  // ---------- Residual (4) ----------
  // ---------- Linear Velocity of Boom ----------
  //double* b_vel = SensorByName(model, data, "boom_vel");
  //mju_copy3(residual + counter, b_vel);
  //counter += 3;
  mju_copy(residual + counter, data->qvel, model->nv);
  counter += model->nv;

  CheckSensorDim(model, counter);

}

// -------- Transition for target --------
//   Follow GoalX by Target
// ---------------------------------------------
void Minicrane::TransitionLocked(mjModel* model, mjData* data) {
    /////data->mocap_pos[0] = parameters[0];
    //...data->mocap_pos[1] = parameters[1];
    /////data->mocap_pos[2] = parameters[2];

    data->mocap_pos[0] = parameters[0];
    data->mocap_pos[1] = parameters[1];
    data->mocap_pos[2] = parameters[2];
    //parameters[3] = count;
    data->mocap_pos[3] = parameters[3];
    count++;
}

}  // namespace mjpc
