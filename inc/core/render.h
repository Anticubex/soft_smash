#include "physics.h"

typedef struct SoftBodyRenderer {
        // A surface following how to render the softbody
        int num;
        int *pts;
        Vector2 *uv;
} SoftBodyRenderer;

void renderSoftbody(SoftBody sb, SoftBodyRenderer rend);

typedef struct PolyNode {
        int *verts;
        struct PolyNode *next;
} PolyNode;

typedef struct Halfplane {
        Vector2 p;
        Vector2 pq;
        float angle;
} Halfplane;

float VecCross(Vector2 a, Vector2 b);
Halfplane createHalfplane(Vector2 a, Vector2 b);
bool outHalfplane(Halfplane plane, Vector2 point);
bool isStarShaped(Vector2 *vertices, int *poly, int num, Vector2 *center);
PolyNode *splitPoly(Vector2 *points, int num);

void merge(Vector2 *points, int *indices, int left, int mid, int right, int *temp);

void mergeSortHelper(Vector2 *points, int *indices, int left, int right, int *temp);

void sortY(Vector2 *points, int num, int *dest);

void DrawTexturePoly(Texture2D texture, Vector2 texcenter, Vector2 center, Vector2 *points, Vector2 *texcoords, int pointCount, Color tint);
