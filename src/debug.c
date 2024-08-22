#include "debug.h"

void DrawSoftbody_debug(SoftBody sb) {
        for (int i = 0; i < sb.numSprings; i++) {
                DrawLineV(sb.pointPos[sb.springA[i]], sb.pointPos[sb.springB[i]], YELLOW);
        }
        DrawLineStrip(sb.pointPos, sb.numPoints, BLACK);
        DrawLineV(sb.pointPos[sb.numPoints - 1], sb.pointPos[0], BLACK);
}