// Copyright by BeeX [2026]
#pragma once

#include <functional>
#include <string>

namespace final_v1 {

// driver and pick log through this; the wrapper decides where it goes.
enum class Level { DEBUG, INFO, WARN, ERROR };
using Log = std::function<void(Level, const std::string &)>;

}  // namespace final_v1
