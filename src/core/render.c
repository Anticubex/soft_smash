#include <core/render.h>
#include <libtess2/tesselator.h>
#include <raylib.h>
#include <stdlib.h>
#include <string.h>

void renderSoftbody(SoftBody sb, SoftBodyRenderer rend) {

        // This alloca means this method is (supposedly) only really "safe"
        // for <512 or so vertices. tbh, if you're using more than that, that's
        // a you issue.
        Vector2 *vertexArray = alloca(sizeof(Vector2) * (rend.num + 1));
        for (int i = 0; i < rend.num; i++) {
                vertexArray[i] = sb.pointPos[rend.pts[i]];
        }
        vertexArray[rend.num] = sb.pointPos[rend.pts[0]]; // For the border spline

        TESStesselator *tessellator = tessNewTess(NULL);
        tessSetOption(tessellator, TESS_CONSTRAINED_DELAUNAY_TRIANGULATION, 1);
        tessAddContour(tessellator, 2, vertexArray, sizeof(Vector2), rend.num);

        tessTesselate(tessellator, TESS_WINDING_ODD, TESS_POLYGONS, 3, 2, NULL);

        int vertexCount = tessGetVertexCount(tessellator);
        const Vector2 *vertices = (const Vector2 *)tessGetVertices(tessellator);
        int indexCount = tessGetElementCount(tessellator) * 3;
        int numTris = tessGetElementCount(tessellator);
        const int *indices = tessGetElements(tessellator);

        for (int t = 0, i = 0; t < numTris; t++, i += 3) {
                DrawTriangle(
                    vertices[indices[i + 2]],
                    vertices[indices[i + 1]],
                    vertices[indices[i + 0]],
                    rend.fillColor);
        }

        DrawSplineLinear(vertexArray, rend.num + 1, rend.thickness, rend.borderColor);

        tessDeleteTess(tessellator);
}

void freeRenderer(SoftBodyRenderer *rend) {
        MemFree(rend->pts);
        rend->num = 0;
}

// Avoid using in prod because this is a quick testing hack
void autogenerateRendererFromSurface(SoftBody sb, SoftBodyRenderer *rend) {
        rend->num = sb.numSurfaces;
        // Assumes the surface is properly connected e.t.c.
        rend->pts = malloc(sizeof(int) * rend->num);
        memcpy(rend->pts, sb.surfaceA, sizeof(int) * rend->num);
}
