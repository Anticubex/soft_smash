#pragma once

#include <cmath>
#include <raylib.h>
#define TAU 6.283185307179586f

class vec2 {
        float x, y;
        vec2(float x = 0.f, float y = 0.f) : x(x), y(y) {};

        vec2 operator+(const vec2 &other) const;
        vec2 operator-(const vec2 &other) const;
        float operator*(const vec2 &other) const;
        vec2 operator*(const float &scalar) const;
        vec2 operator/(const float &scalar) const;

        float mag() const;
        float magSqr() const;
        float invMag() const;

        vec2 normalized() const;
        vec2 normalize();
};

float fastInvSqrt(float x);
