#pragma once

#include <cmath>

#include "math.hpp"

namespace rhoban
{
    /**
     * Brensenham's line algorithm implementation as an iterator
     */
    class LineIterator
    {
    private:
        math::Point<int> current;
        math::Point<int> start, end;

        int dx, dy; // Difference between x start/end and y respectively
        int x_inc_uncond, y_inc_uncond;
        int x_inc_cond, y_inc_cond;

        int error, slope, errorInc;

        int current_pixel, num_pixels;

    public:
        LineIterator(math::Point<int> start_, math::Point<int> end_)
            : current(start_), start(start_), end(end_), current_pixel(0)
        {
            int xtemp = end.x - start.x;
            int ytemp = end.y - start.y;

            dx = abs(xtemp);
            dy = abs(ytemp);

            x_inc_uncond = math::sign(xtemp);
            y_inc_uncond = math::sign(ytemp);

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

        ~LineIterator();

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
        }

        math::Point<int> getCurrent()
        {
            return current;
        }

        math::Point<int> getStart()
        {
            return start;
        }

        math::Point<int> getEnd()
        {
            return end;
        }
    };

} // namespace rhoban
