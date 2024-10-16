#include "debug.h"
#include <stdlib.h>

inline float I3WHF(unsigned char a, unsigned char b, float t) __attribute__((always_inline));
float I3WHF(unsigned char a, unsigned char b, float t) {
        return a + (b - a) * t;
}

Color interpolate3way(Color A, Color B, Color C, float t) {
        if (t <= -1.f)
                return A;
        else if (t >= 1.f)
                return C;
        else if (t < 0.f)
                return (Color){
                    .r = I3WHF(A.r, B.r, t + 1.f),
                    .g = I3WHF(A.g, B.g, t + 1.f),
                    .b = I3WHF(A.b, B.b, t + 1.f),
                    .a = I3WHF(A.a, B.a, t + 1.f),
                };
        return (Color){
            .r = I3WHF(B.r, C.r, t),
            .g = I3WHF(B.g, C.g, t),
            .b = I3WHF(B.b, C.b, t),
            .a = I3WHF(B.a, C.a, t),
        };
}

void DrawSoftbody_debug(SoftBody sb) {
        // // Draw shape
        // Matrix shapeMatrix = MatrixIdentity();
        // float sinA = sinf(sb.shapeRotation);
        // float cosA = cosf(sb.shapeRotation);
        // shapeMatrix.m0 = cosA;
        // shapeMatrix.m1 = sinA;
        // shapeMatrix.m4 = -sinA;
        // shapeMatrix.m5 = cosA;
        // Vector2 *shapePos = alloca(sizeof(Vector2) * sb.numPoints);
        // for (int i = 0; i < sb.numPoints; i++) {
        //         shapePos[i] = Vector2Add(Vector2Transform(sb.shape[i], shapeMatrix), sb.shapePosition);
        // }
        // for (int i = 0; i < sb.numPoints; i++) {
        //         // Line
        //         DrawLineEx(shapePos[i], shapePos[(i + 1) % sb.numPoints], 0.1f, GRAY);
        //         DrawLineEx(shapePos[i], sb.pointPos[i], 0.15f, YELLOW);
        // }
        // // Draw Springs
        // for (int i = 0; i < sb.numSprings; i++) {
        //         DrawLineEx(sb.pointPos[sb.springA[i]], sb.pointPos[sb.springB[i]], 0.15f,
        //                    interpolate3way(RED, GREEN, RED, sb.lengths[i] - Vector2Distance(sb.pointPos[sb.springA[i]], sb.pointPos[sb.springB[i]])));
        // }

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