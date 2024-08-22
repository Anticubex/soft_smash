#include "debug.h"

void DrawSoftbody_debug(SoftBody sb) {
        for (int i = 0; i < sb.numSprings; i++) {
                DrawLineEx(sb.pointPos[sb.springA[i]], sb.pointPos[sb.springB[i]], 0.1f, YELLOW);
        }
        // DrawLineStrip(sb.pointPos, sb.numPoints, BLACK);
        // DrawLineV(sb.pointPos[sb.numPoints - 1], sb.pointPos[0], BLACK);
        for (int i = 0; i < sb.numPoints; i++) {
                DrawLineEx(sb.pointPos[i], sb.pointPos[(i + 1) % sb.numPoints], 0.05f, BLACK);
                DrawCircleV(sb.pointPos[i], 0.1f, BLACK);
                DrawTextEx(GetFontDefault(), TextFormat("%i", i), Vector2Add(sb.pointPos[i]), 0.1f, 0.0f, WHITE);
        }
}