// Copyright © 2019-2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Static dispatcher: statically-linked replacement for the dlopen-based
// stub (sw/runtime/stub/vortex.cpp) on platforms without dynamic loading
// (static musl binaries, bare metal). The zynq backend's vx_dev_init is
// linked into the same image and used unconditionally.

#include "dispatcher.h"

#include <mutex>

namespace vx {

vx_result_t dispatcher_get_callbacks(const callbacks_t** out) {
  static callbacks_t table;
  static vx_result_t status = []() -> vx_result_t {
    return (0 == vx_dev_init(&table)) ? VX_SUCCESS : VX_ERR_DEVICE_LOST;
  }();
  if (status != VX_SUCCESS)
    return status;
  *out = &table;
  return VX_SUCCESS;
}

} // namespace vx
