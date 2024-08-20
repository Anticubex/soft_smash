#include <core.h>
#include <math.h>
#include <raylib.h>
#include <raymath.h>

Vector2 applyTransform(Transform2D transform, Vector2 v) {
        float sinA = sin(transform.rotation);
        float cosA = cos(transform.rotation);
        return (Vector2){
            transform.position.x + (v.x * cosA - v.y * sinA),
            transform.position.y + (v.x * sinA + v.y * cosA),
        };
}
