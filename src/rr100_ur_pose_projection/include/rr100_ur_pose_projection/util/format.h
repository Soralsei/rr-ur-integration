#pragma once

#include <string>
#include <cstdarg>
#include <vector>

namespace rhoban
{
    // requires at least C++11
    const std::string format(const char *const zcFormat, ...);

} // namespace rhoban