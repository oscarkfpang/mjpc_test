// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MJPC_APP_H_
#define MJPC_APP_H_

#include <memory>
#include <vector>

#include "mjpc/simulate.h"  // mjpc fork
#include "mjpc/task.h"

using ClockT = std::chrono::_V2::steady_clock;
using Seconds = std::chrono::duration<double>;
using TimePoint = std::chrono::time_point<ClockT>;

namespace mjpc {
class MjpcApp {
 public:
  MjpcApp(std::vector<std::shared_ptr<mjpc::Task>> tasks, int task_id = 0);
  MjpcApp(const MjpcApp&) = delete;
  MjpcApp& operator=(const MjpcApp&) = delete;
  ~MjpcApp();

  void Start();

  ::mujoco::Simulate* Sim();

 // for syncing ROS time with sim time
 private:
  std::chrono::time_point<ClockT> syncROS;
  std::chrono::time_point<ClockT> start_time;
  std::atomic_flag time_initialized;
};

void StartApp(std::vector<std::shared_ptr<mjpc::Task>> tasks, int task_id = 0);

}  // namespace mjpc

#endif  // MJPC_APP_H_
