#include <core/render.h>
#include <math.h>
#include <raylib.h>
#include <rlgl.h>
#include <stdio.h>
#include <stdlib.h>

void renderSoftbody(SoftBody sb, SoftBodyRenderer rend) {
}

float VecCross(Vector2 a, Vector2 b) {
        return a.x * b.y - a.y * b.x;
}

Halfplane createHalfplane(Vector2 a, Vector2 b) {
        Vector2 pq = Vector2Subtract(b, a);
        return (Halfplane){
            .p = a,
            .pq = pq,
            .angle = atan2(pq.y, pq.x),
        };
}

int compareHalfplanes(const void *vpa, const void *vpb) {
        Halfplane *a = vpa;
        Halfplane *b = vpb;
        return (a->angle > b->angle) - (a->angle < b->angle);
}

bool outHalfplane(Halfplane plane, Vector2 point) {
        return VecCross(plane.pq, Vector2Subtract(point, plane.p)) < 0.f;
}

Vector2 intersection(Halfplane a, Halfplane b) {
        float alpha = VecCross(Vector2Subtract(b.p, a.p), b.pq) /
                      VecCross(a.pq, b.pq);
        return Vector2Add(a.p, Vector2Scale(a.pq, alpha));
}

bool isStarShaped(Vector2 *vertices, int *poly, int num, Vector2 *center) {
        // First, get the intersection of all the halfplanes

        // Generate the halfplanes
        Halfplane *planes = alloca(sizeof(Halfplane) * num);

        // planes[0] = createHalfplane((Vector2){FP_INFINITE, FP_INFINITE}, (Vector2){-FP_INFINITE, FP_INFINITE});
        // planes[1] = createHalfplane((Vector2){-FP_INFINITE, FP_INFINITE}, (Vector2){-FP_INFINITE, -FP_INFINITE});
        // planes[2] = createHalfplane((Vector2){-FP_INFINITE, -FP_INFINITE}, (Vector2){FP_INFINITE, -FP_INFINITE});
        // planes[3] = createHalfplane((Vector2){FP_INFINITE, -FP_INFINITE}, (Vector2){FP_INFINITE, FP_INFINITE});

        for (int i = 0; i < num; i++) {

                planes[i] = createHalfplane(vertices[poly[i]], vertices[poly[(i + 1) % num]]);
        }

        // sort them and stuff
        qsort(planes, num, sizeof(Halfplane), compareHalfplanes);
        int *deque = alloca(sizeof(int) * num);
        int front = 0, back = 0, len = 1;

        deque[0] = 0;

        for (int i = 1; i < num; i++) {

                // Remove from the back of the deque while last half-plane is redundant and
                // the front of the deque while the first half-plane is redundant.
                // * Note that at any step len>=2, so just incrementing/decrementing is safe.
                while (len > 1 &&
                       outHalfplane(planes[i], intersection(planes[deque[back]],
                                                            planes[deque[back - 1]]))) {
                        back--;
                        len--;
                }
                while (len > 1 &&
                       outHalfplane(planes[i], intersection(planes[deque[front]],
                                                            planes[deque[front + 1]]))) {
                        front++;
                        len--;
                }

                // Special case check: Parallel half-planes
                // * Note: This is graphics, so epsilon doesn't have to be perfect
                if (len > 0 && fabsl(VecCross(planes[i].pq, planes[back].pq)) < 0.00001F) {
                        // Opposite parallel half-planes that ended up checked against each other.
                        if (Vector2DotProduct(planes[i].pq, planes[deque[back]].pq) < 0.0)
                                return false;

                        // Same direction half-plane: keep only the leftmost half-plane.
                        if (outHalfplane(planes[i], planes[deque[back]].p)) {
                                back--; // Again, len > 1 so safe (or at least it will be after pushing)
                                len--;
                        } else
                                continue;
                }

                // Add new half-plane
                deque[++back] = i;
                ++len;
        }

        // Final cleanup: Check half-planes at the front against the back and vice-versa
        while (len > 2 && outHalfplane(planes[deque[0]], intersection(planes[deque[back]], planes[deque[back - 1]]))) {
                back--;
                len--;
        }
        while (len > 2 && outHalfplane(planes[deque[back]], intersection(planes[deque[front]], planes[deque[front + 1]]))) {
                front++;
                len--;
        }

        if (len < 3)
                return false;

        // Average out the intersection points to get a nice center

        *center = (Vector2){0.f, 0.f};
        for (int i = front; i < back; i++) {
                *center = Vector2Add(*center, intersection(planes[deque[i]], planes[deque[i + 1]]));
        }
        *center = Vector2Add(*center, intersection(planes[deque[back]], planes[deque[front]]));
        *center = Vector2Scale(*center, 1.f / len);

        return 0;
}

// Checks if the point n is the start of an ear
bool isTri(Vector2 *points, int num, int n) {
        // The most common case for it *not* being an edge is when the
        // vertex n+1 is concave; That is, like so:
        //* n-n+1
        //*      \
        //*       n+2
        // So we check that first

        Vector2 a = points[n];
        Vector2 b = points[(n + 1) % num];
        Vector2 c = points[(n + 2) % num];

        Vector2 e1 = Vector2Subtract(b, a);
        Vector2 e2 = Vector2Subtract(c, b);

        if (VecCross(e1, e2) <= 0.f)
                return false;

        // Now we check if the cutting edge crosses any sides of the polygon
        // Or don't and let them suffer with it because this case is probably
        // exceedingly rare i.e. "ostrich optimization"
        return true;
}

// Splits a polygon into a list of triangles
PolyNode *splitPoly(Vector2 *points, int num) {
        // Idea: keep chunking off triangles off the edges until the remnants are star-shaped
        // The star-shaped check helps to keep the texture from being disgustingly squished
        // I think this is basically an application of the two ears theorem

        // Here we see an optimization tweak: how many ears do we chop off
        // until we check for a star shape again?
}

// Draw textured polygon, defined by vertex and texture coordinates
// NOTE: Polygon center must have straight line path to all points
// without crossing perimeter, points must be in anticlockwise order
// i.e. star shaped w/ `center` in kernel
void DrawTexturePoly(Texture2D texture, Vector2 texcenter, Vector2 center, Vector2 *points, Vector2 *texcoords, int pointCount, Color tint) {
        rlSetTexture(texture.id);

        // Texturing is only supported on RL_QUADS
        rlBegin(RL_QUADS);

        rlColor4ub(tint.r, tint.g, tint.b, tint.a);

        for (int i = 0; i < pointCount - 1; i++) {
                rlTexCoord2f(texcenter.x, texcenter.y);
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
