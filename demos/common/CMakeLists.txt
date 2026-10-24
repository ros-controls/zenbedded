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

if(CONFIG_ZENBEDDED_DEMO_COMMON)
  zephyr_library_named(zenbedded_demo_common)
  zephyr_library_sources(
    src/demo_network.cpp
  )
  zephyr_include_directories(include)

  zephyr_library_sources_ifdef(CONFIG_USB_DEVICE_STACK_NEXT
    src/demo_usbd.c
  )
endif()
