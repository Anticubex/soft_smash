#include "debug.h"

void DrawSoftbody_debug(SoftBody sb) {
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