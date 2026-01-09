#include <bettermath.hpp>

vec2 vec2::operator+(const vec2 &other) const {
        return vec2(x + other.x, y + other.y);
}

vec2 vec2::operator-(const vec2 &other) const {
        return vec2(x - other.x, y - other.y);
}

float vec2::operator*(const vec2 &other) const {
        return x * other.x + y * other.y;
}

vec2 vec2::operator*(const float &scalar) const {
        return vec2(x * scalar, y * scalar);
}

vec2 vec2::operator/(const float &scalar) const {
        return vec2(x / scalar, y / scalar);
}

float vec2::mag() const {
        return sqrtf(magSqr());
}
float vec2::magSqr() const {
        return x * x + y * y;
}
float vec2::invMag() const {
}

float fastInvSqrt(float x) {
        return 0.0f;
}
