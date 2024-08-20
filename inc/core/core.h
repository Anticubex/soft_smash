#ifndef CORE_H
#define CORE_H

#include <raylib.h>
#include <raymath.h>

typedef struct {
        Vector2 position;
        float rotation;
} Transform2D;

Vector2 applyTransform(Transform2D transform, Vector2 v);

#endif