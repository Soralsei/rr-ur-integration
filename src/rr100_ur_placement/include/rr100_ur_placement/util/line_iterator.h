#pragma once

#include <cmath>
#include <Eigen/Dense>

// #include "math.h"

namespace rhoban
{
    /**
     * Brensenham's line algorithm implementation as an iterator
     */
    class LineIterator
    {
    private:
        Eigen::Vector2i current;
        Eigen::Vector2i start, end;

        Eigen::Vector2i dPos;       // Difference between x start/end and y respectively
        Eigen::Vector2i inc_uncond;
        Eigen::Vector2i inc_cond;
        // int x_inc_uncond, y_inc_uncond;
        // int x_inc_cond, y_inc_cond;

        int error, slope, errorInc;

        int current_pixel, num_pixels;

    public:
        LineIterator(const Eigen::Vector2i &start_, const Eigen::Vector2i &end_)
            : current(start_), start(start_), end(end_), current_pixel(0), error(0), slope(0), errorInc(0)
        {
            Eigen::Vector2i tmp = end - start;
            dPos = tmp.cwiseAbs();
            inc_uncond = tmp.cwiseSign();
            inc_cond = inc_uncond;

            num_pixels = dPos.maxCoeff();

            reset();
        }

        ~LineIterator() {}

        void reset() {
            current_pixel = 0;
            current = start;
            // Octant 0/3 (more vertical than horizontal)
            if (dPos[0] >= dPos[1])
            {
                inc_cond[0] = 0;
                inc_uncond[1] = 0;

                slope = 2 * dPos[1];
                error = -dPos[0];
                errorInc = -2 * dPos[0];

            }
            // Octant 1/2 (more vertical than horizontal)
            else
            {
                inc_uncond[0] = 0;
                inc_cond[1] = 0;

                slope = 2 * dPos[0];
                error = -dPos[1];
                errorInc = -2 * dPos[1];
            }
        }

        bool endReached()
        {
            return current_pixel >= num_pixels;
        }

        void next()
        {
            error += slope;
            if (error >= 0)
            {
                current += inc_cond;
                error += errorInc;
            }
            current += inc_uncond;
            current_pixel++;
        }

        int getX()
        {
            return current[0];
        }

        int getY()
        {
            return current[1];
        }

        Eigen::Vector2i getCurrent()
        {
            return current;
        }

        Eigen::Vector2i getStart()
        {
            return start;
        }

        Eigen::Vector2i getEnd()
        {
            return end;
        }
    };

} // namespace rhoban
