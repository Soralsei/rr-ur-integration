#pragma once

#include <cmath>

#include "math.h"

namespace rhoban
{
    /**
     * Brensenham's line algorithm implementation as an iterator
     */
    class LineIterator
    {
    private:
        math::Vector2<int> current;
        math::Vector2<int> start, end;

        int dx, dy; // Difference between x start/end and y respectively
        int x_inc_uncond, y_inc_uncond;
        int x_inc_cond, y_inc_cond;

        int error, slope, errorInc;

        int current_pixel, num_pixels;

    public:
        LineIterator(const math::Vector2<int> &start_, const math::Vector2<int> &end_)
            : current(start_), start(start_), end(end_), current_pixel(0), error(0), slope(0), errorInc(0)
        {
            int xtemp = end.x - start.x;
            int ytemp = end.y - start.y;

            dx = abs(xtemp);
            dy = abs(ytemp);

            x_inc_uncond = math::sign(xtemp);
            y_inc_uncond = math::sign(ytemp);

            x_inc_cond = x_inc_uncond;
            y_inc_cond = y_inc_uncond;

            // Octant 0-3 (more horizontal than vertical)
            if (dx >= dy)
            {
                x_inc_cond = 0;
                y_inc_uncond = 0;

                slope = 2 * dy;
                error = -dx;
                errorInc = -2 * dx;

                num_pixels = dx;
            } 
            // Octant 1-2 (more vertical than horizontal)
            else
            {
                x_inc_uncond = 0;
                y_inc_cond = 0;

                slope = 2 * dx;
                error = -dy;
                errorInc = -2 * dy;

                num_pixels = dy;
            }
        }

        ~LineIterator() {}

        bool endReached()
        {
            return current_pixel >= num_pixels;
        }

        void next()
        {
            error += slope;
            if (error >= 0)
            {
                current.x += x_inc_cond;
                current.y += y_inc_cond;
                error += errorInc;
            }
            current.x += x_inc_uncond;            
            current.y += y_inc_uncond;
            current_pixel++;         
        }

        math::Vector2<int> getCurrent()
        {
            return current;
        }

        math::Vector2<int> getStart()
        {
            return start;
        }

        math::Vector2<int> getEnd()
        {
            return end;
        }
    };

} // namespace rhoban
