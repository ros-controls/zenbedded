// Copyright 2026 Open Source Robotics Foundation, Inc.
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

// Per-checkpoint config provenance - C++ port of the Python `run_config.py`
// idea: a checkpoint records the knobs that MUST match between whatever
// trained/saved it and whatever loads it (here: the state layout and action
// scaling), so a checkpoint loaded with mismatched settings fails loudly
// instead of behaving nonsensically on the rig. The binary weight files
// already refuse to load on a state_dim mismatch (see actor_critic.hpp's
// kFormatVersion check) - this is the human-readable second layer that also
// covers knobs the binary format doesn't encode (action scaling, reward
// shape) and produces an actual diagnostic message instead of a bare
// load-failed.
//
// Deliberately NOT JSON: this project stays dependency-free (see
// tiny_nn.hpp's header comment), so this is a flat `key=value` text file,
// one entry per line.
#pragma once

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace inverted_pendulum_rl_controller
{
namespace run_config
{

using ConfigMap = std::map<std::string, std::string>;

constexpr const char * kSharedConfigName = "config.txt";

// Keys recorded for provenance but NOT enforced by check(): they don't
// change the observation/action layout or the training objective, so a
// checkpoint trained one way can legitimately be loaded and run the other
// way. Mirrors PROVENANCE_ONLY_KEYS in the Python run_config.py - mirror
// augmentation is the same example: turning it on for a later curriculum
// stage/resume is normal and must not abort a load.
inline const std::vector<std::string> & provenance_only_keys()
{
  static const std::vector<std::string> keys = {
    "mirror_augmentation", "gamma", "tau", "actor_lr", "critic_lr"};
  return keys;
}

inline bool is_provenance_only(const std::string & key)
{
  const auto & keys = provenance_only_keys();
  return std::find(keys.begin(), keys.end(), key) != keys.end();
}

inline bool save(const std::filesystem::path & path, const ConfigMap & config)
{
  std::ofstream f(path);
  if (!f)
  {
    return false;
  }
  for (const auto & [k, v] : config)
  {
    f << k << "=" << v << "\n";
  }
  return static_cast<bool>(f);
}

inline std::optional<ConfigMap> load(const std::filesystem::path & path)
{
  std::ifstream f(path);
  if (!f)
  {
    return std::nullopt;
  }
  ConfigMap config;
  std::string line;
  while (std::getline(f, line))
  {
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    const auto eq = line.find('=');
    if (eq == std::string::npos)
    {
      continue;
    }
    config[line.substr(0, eq)] = line.substr(eq + 1);
  }
  return config;
}

// Looks for `<checkpoint_stem>.config.txt` next to the checkpoint first
// (per-file provenance), then falls back to a directory-wide `config.txt`.
// Checkpoints predating this mechanism have neither - callers should treat
// std::nullopt as "nothing to validate against", not an error (same as
// find_run_config() returning None in the Python version).
inline std::optional<ConfigMap> find_run_config(const std::filesystem::path & checkpoint_path)
{
  const auto sibling =
    checkpoint_path.parent_path() / (checkpoint_path.stem().string() + ".config.txt");
  if (auto cfg = load(sibling))
  {
    return cfg;
  }
  return load(checkpoint_path.parent_path() / kSharedConfigName);
}

struct CheckResult
{
  std::vector<std::string> mismatches;  // one human-readable line per bad key
  bool ok() const { return mismatches.empty(); }
};

// Compares `expected` (the values this run is about to use) against a
// checkpoint's saved config. Skips provenance-only keys and any key absent
// from either side - a checkpoint predating a given key can't be validated
// against it, same reasoning as the Python check_config().
inline CheckResult check(const ConfigMap & saved, const ConfigMap & expected)
{
  CheckResult result;
  for (const auto & [key, expected_val] : expected)
  {
    if (is_provenance_only(key))
    {
      continue;
    }
    const auto it = saved.find(key);
    if (it == saved.end())
    {
      continue;
    }
    const std::string & saved_val = it->second;
    bool same;
    try
    {
      same = std::fabs(std::stod(saved_val) - std::stod(expected_val)) < 1e-9;
    }
    catch (const std::exception &)
    {
      same = (saved_val == expected_val);
    }
    if (!same)
    {
      result.mismatches.push_back(
        "  " + key + ": checkpoint recorded '" + saved_val + "', current config gives '" +
        expected_val + "'");
    }
  }
  return result;
}

}  // namespace run_config
}  // namespace inverted_pendulum_rl_controller
