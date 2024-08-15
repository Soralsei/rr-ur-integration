#pragma once

namespace rhoban
{
    namespace math
    {

        template <typename T>
        struct Vector2
        {
            T x;
            T y;

            Vector2<T> operator-() const
            {
                Vector2<T> point;
                point.x = -x;
                point.y = -y;

                return point;
            }

            Vector2<T> operator*(const Vector2<T> &other) const
            {
                Vector2<T> point;
                point.x = x * other.x;
                point.y = y * other.y;

                return point;
            }

            Vector2<T> operator+(const Vector2<T> &other) const
            {
                Vector2<T> point;
                point.x = x + other.x;
                point.y = y + other.y;

                return point;
            }

            Vector2<T> operator-(const Vector2<T> &other) const
            {
                Vector2<T> point;
                point.x = x - other.x;
                point.y = y - other.y;

                return point;
            }

            Vector2<T> operator*(const T &scalar) const
            {
                Vector2<T> point;
                point.x = x * scalar;
                point.y = y * scalar;

                return point;
            }

            Vector2<T> operator/(const T &scalar) const
            {
                Vector2<T> point;
                point.x = x / scalar;
                point.y = y / scalar;

                return point;
            }

            Vector2<T> operator+(const T &scalar) const
            {
                Vector2<T> point;
                point.x = x + scalar;
                point.y = y + scalar;

                return point;
            }

            Vector2<T> operator-(const T &scalar) const
            {
                Vector2<T> point;
                point.x = x - scalar;
                point.y = y - scalar;

                return point;
            }
        };


        int inline sign(int n)
        {
            return n > 0 ? 1 : (n < 0 ? -1 : 0);
        }

    } // namespace math

} // namespace rhoban
