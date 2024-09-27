#include <core/render.h>
#include <raylib.h>
#include <rlgl.h>
#include <stdlib.h>

void renderSoftbody(SoftBody sb, SoftBodyRenderer rend) {
}

// void _splitPoly_internal(Vector2 *points, int num, int *retNum, )

void merge(Vector2 *points, int *indices, int left, int mid, int right, int *temp) {
        int i = left, j = mid + 1, k = 0;

        while (i <= mid && j <= right) {
                if (points[indices[i]].y <= points[indices[j]].y) {
                        temp[k++] = indices[i++];
                } else {
                        temp[k++] = indices[j++];
                }
        }

        while (i <= mid) {
                temp[k++] = indices[i++];
        }

        while (j <= right) {
                temp[k++] = indices[j++];
        }

        for (i = 0; i < k; i++) {
                indices[left + i] = temp[i];
        }
}

void mergeSortHelper(Vector2 *points, int *indices, int left, int right, int *temp) {
        if (left < right) {
                int mid = left + (right - left) / 2;
                mergeSortHelper(points, indices, left, mid, temp);
                mergeSortHelper(points, indices, mid + 1, right, temp);
                merge(points, indices, left, mid, right, temp);
        }
}

void sortY(Vector2 *points, int num, int *dest) {
        int *temp = alloca(num * sizeof(int));

        // Initialize dest with indices
        for (int i = 0; i < num; i++) {
                dest[i] = i;
        }

        mergeSortHelper(points, dest, 0, num - 1, temp);
}

// Splits a polygon into a list of triangles
PolyNode *splitPoly(Vector2 *points, int num) {
        // Idea: keep chunking off triangles off the edges until the remnants are star-shaped
        // I think this is basically an application of the two ears theorem
}

// Draw textured polygon, defined by vertex and texture coordinates
// NOTE: Polygon center must have straight line path to all points
// without crossing perimeter, points must be in anticlockwise order
// i.e. star shaped w/ `center` in kernel
void DrawTexturePoly(Texture2D texture, Vector2 center, Vector2 *points, Vector2 *texcoords, int pointCount, Color tint) {
        rlSetTexture(texture.id);

        // Texturing is only supported on RL_QUADS
        rlBegin(RL_QUADS);

        rlColor4ub(tint.r, tint.g, tint.b, tint.a);

        for (int i = 0; i < pointCount - 1; i++) {
                rlTexCoord2f(0.5f, 0.5f);
                rlVertex2f(center.x, center.y);

                rlTexCoord2f(texcoords[i].x, texcoords[i].y);
                rlVertex2f(points[i].x + center.x, points[i].y + center.y);

                rlTexCoord2f(texcoords[i + 1].x, texcoords[i + 1].y);
                rlVertex2f(points[i + 1].x + center.x, points[i + 1].y + center.y);

                rlTexCoord2f(texcoords[i + 1].x, texcoords[i + 1].y);
                rlVertex2f(points[i + 1].x + center.x, points[i + 1].y + center.y);
        }
        rlEnd();

        rlSetTexture(0);
}
