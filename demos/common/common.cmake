# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

get_filename_component(ZENBEDDED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

list(APPEND ZEPHYR_EXTRA_MODULES
  "${ZENBEDDED_ROOT}/zenbedded_transport"
  "${ZENBEDDED_ROOT}/zenbedded_rcl"
  "${ZENBEDDED_ROOT}/demos/common"
)

list(APPEND EXTRA_CONF_FILE
  "${CMAKE_CURRENT_LIST_DIR}/prj_common.conf"
)
