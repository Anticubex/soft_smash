#include "debug.h"
#include <stdlib.h>

void DrawSoftbody_debug(SoftBody sb) {
        // Draw shape

        Matrix shapeMatrix = MatrixIdentity();
        float sinA = sinf(sb.shapeRotation);
        float cosA = cosf(sb.shapeRotation);
        shapeMatrix.m0 = cosA;
        shapeMatrix.m1 = sinA;
        shapeMatrix.m4 = -sinA;
        shapeMatrix.m5 = cosA;
        Vector2 *shapePos = alloca(sizeof(Vector2) * sb.numPoints);
        for (int i = 0; i < sb.numPoints; i++) {
                shapePos[i] = Vector2Add(Vector2Transform(sb.shape[i], shapeMatrix), sb.shapePosition);
        }
        for (int i = 0; i < sb.numPoints; i++) {
                // Line
                DrawLineEx(shapePos[i], shapePos[(i + 1) % sb.numPoints], 0.1f, GRAY);
                DrawLineEx(shapePos[i], sb.pointPos[i], 0.15f, YELLOW);
        }
        // Draw Springs
        for (int i = 0; i < sb.numSprings; i++) {
                DrawLineEx(sb.pointPos[sb.springA[i]], sb.pointPos[sb.springB[i]], 0.15f, YELLOW);
        }
        // Draw surfaces
        for (int i = 0; i < sb.numSurfaces; i++) {
                DrawLineEx(sb.pointPos[sb.surfaceA[i]], sb.pointPos[sb.surfaceB[i]], 0.1f, BLACK);
        }
        // Draw/label points
        for (int i = 0; i < sb.numPoints; i++) {
                DrawCircleV(sb.pointPos[i], 0.2f, BLACK);
                DrawTextEx(GetFontDefault(), TextFormat("%i", i), Vector2SubtractValue(sb.pointPos[i], 0.1f), 0.2f, 0.0f, WHITE);
        }
}