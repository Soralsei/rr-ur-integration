#pragma once

namespace rhoban
{
    
namespace math
{
    
    template<typename T>
    struct Point
    {
        T x;
        T y;
    };

    int inline sign(int n) {
        return n > 0 ? 1 : (n < 0 ? -1 : 0);
    }

} // namespace math

} // namespace rhoban
