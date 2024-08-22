#include <assert.h>
#include <bettermath.h>
#include <core/physics.h>
#include <stddef.h>

void update_SoftBody(SoftBody *sb, WorldValues worldValues, float dt) {
        // Now for the behemoth

        // Update every point
        for (int i = 0; i < sb->numPoints; i++) {
                sb->pointPos[i] = Vector2Add(sb->pointPos[i], Vector2Scale(sb->pointVel[i], dt));
                sb->pointVel[i] = Vector2Add(sb->pointVel[i], Vector2Scale(worldValues.gravity, dt));
        }
        // Update bounding box
        float minx = sb->pointPos[0].x;
        float maxx = minx;
        float miny = sb->pointPos[0].y;
        float maxy = miny;
        for (int i = 1; i < sb->numPoints; i++) {
                Vector2 pos = sb->pointPos[i];
                minx = pos.x < minx ? pos.x : minx;
                maxx = pos.x > maxx ? pos.x : maxx;
                miny = pos.y < miny ? pos.y : miny;
                maxy = pos.y > maxy ? pos.y : maxy;
        }
        sb->bounds.min = (Vector2){minx, miny};
        sb->bounds.max = (Vector2){maxx, maxy};

        if (sb->type & SoftBodyType_Shape) {
                // Recalculate frame center and rotation
                Vector2 avg_pos = {0, 0};
                for (int i = 0; i < sb->numPoints; i++) {
                        avg_pos.x += sb->pointPos[i].x;
                        avg_pos.y += sb->pointPos[i].y;
                }
                avg_pos = Vector2Scale(avg_pos, 1 / sb->numPoints);
                float avg_rotation = 0;
                for (int i = 0; i < sb->numPoints; i++) {
                        avg_rotation += Vector2Angle(Vector2Add(sb->shape[i], avg_pos), sb->pointPos[i]);
                }
                avg_rotation /= sb->numPoints;

                Matrix shapeMatrix = MatrixIdentity();
                float sinA = sinf(avg_rotation);
                float cosA = cosf(avg_rotation);
                shapeMatrix.m0 = cosA;
                shapeMatrix.m1 = sinA;
                shapeMatrix.m4 = -sinA;
                shapeMatrix.m5 = cosA;

                for (int i = 0; i < sb->numPoints; i++) {
                        // Calculate distance
                        Vector2 shape_pos = Vector2Add(Vector2Transform(sb->shape[i], shapeMatrix), avg_pos);
                        Vector2 diff = Vector2Subtract(shape_pos, sb->pointPos[i]);
                        float f = Vector2Length(diff) * sb->shapeSpringStrength;
                        SBPoint_addForce(sb, i, Vector2Scale(Vector2Normalize(diff), f), dt);
                }
        }
        // Note: trying to do pressure on a softbody without at least 2 points leads to a segfault.
        // Hmm. Blame it on the user.
        if (sb->type & SoftBodyType_Pressure) {
                // Calculate Volume
                float V =
                    sb->pointPos[0].y * (sb->pointPos[sb->numPoints - 1].x - sb->pointPos[1].x) +
                    sb->pointPos[sb->numPoints - 1].y * (sb->pointPos[sb->numPoints - 2].x - sb->pointPos[0].x);
                // Do edge cases
                for (int i = 1; i < sb->numPoints - 1; i++) {
                        V += sb->pointPos[i].y * (sb->pointPos[i - 1].x - sb->pointPos[i + 1].x);
                }
                V *= 0.5;
                // Use gas law for pressure
                float P = sb->nRT / V;

                // Apply Pressure
                for (int i = 0; i < sb->numPoints; i++) {
                        Vector2 a = sb->pointPos[i];
                        Vector2 b = sb->pointPos[(i + 1) % sb->numPoints];
                        Vector2 diff = Vector2Subtract(a, b);

                        // Multiplying P by the difference saves us having to calculate the length
                        // This is where the CCW winding comes in

                        Vector2 normal = {-diff.y * P, diff.x * P};

                        SBPoint_addForce(sb, i, normal, dt);
                        SBPoint_addForce(sb, (i + 1) % sb->numPoints, normal, dt);
                }
        }

        // Now for your basic spring force
        for (int i = 0; i < sb->numSprings; i++) {
                int a_idx = sb->springA[i];
                int b_idx = sb->springB[i];
                Vector2 a_pos = sb->pointPos[a_idx];
                Vector2 b_pos = sb->pointPos[b_idx];
                Vector2 a_vel = sb->pointVel[a_idx];
                Vector2 b_vel = sb->pointVel[b_idx];
                Vector2 diff = Vector2Subtract(a_pos, b_pos);
                // b -> a

                // Calculate force
                // f = -kx - cx'

                float length = Vector2Length(diff);
                Vector2 diffNorm = Vector2Scale(diff, 1. / length);
                float x = length - sb->lengths[i];

                float f = -sb->springStrength * x -
                          sb->springDamp * Vector2DotProduct(Vector2Subtract(a_vel, b_vel), diffNorm);

                SBPoint_addForce(sb, a_idx, Vector2Scale(diffNorm, f), dt);
                SBPoint_addForce(sb, b_idx, Vector2Scale(diffNorm, -f), dt);
        }

        // TODO : Softbody drag/air resistance, using area and stuff
}

void SBPoint_addForce(SoftBody *sb, int i, Vector2 force, float dt) {
        sb->pointVel[i] = Vector2Add(sb->pointVel[i], Vector2Scale(force, dt));
}

Vector2 rotateQuarterTurn(Vector2 v) {
        return (Vector2){-v.y, v.x};
}

// Recommended to just do pressure
void circleSoftbody(SoftBody *sb, Vector2 center, float radius, int numPoints) {

        float anglePer = TAU / (float)numPoints;
        float sinA = sin(anglePer);
        float cosA = cos(anglePer);
        // In-house rotation matrix from rows
        Vector2 row1 = {cosA, -sinA};
        Vector2 row2 = {sinA, cosA};

        float lengths = hypotf(cosA - 1, sinA) * radius;

        _alloc_sb(sb, numPoints, numPoints, numPoints);

        Vector2 tracker = {radius, 0.f};
        for (int i = 0; i < numPoints; i++) {
                sb->pointPos[i] = Vector2Add(tracker, center);
                sb->pointVel[i] = Vector2Zero();
                sb->shape[i] = tracker;
                sb->surfaceA[i] = i;
                sb->surfaceB[i] = i + 1;
                sb->springA[i] = i;
                sb->springB[i] = i + 1;
                sb->lengths[i] = lengths;
                tracker = (Vector2){Vector2DotProduct(tracker, row1),
                                    Vector2DotProduct(tracker, row2)};
        }
        sb->surfaceB[numPoints - 1] = 0;
        sb->springB[numPoints - 1] = 0;
}

// Recommended to just do shape matching.
// Pressure could be bad.
// DetailX and Y should be > 1, as they are the
// number of subdivisions of sides in either dimension
void rectSoftbody(SoftBody *sb, Vector2 center, Vector2 scale, int detailX, int detailY, bool makeTruss) {

        assert(detailX > 1);
        assert(detailY > 1);

        if (makeTruss) {
                int px = detailX + 1;
                int py = detailY + 1;
                int numPoints = px * py;

                // The number of links is a math problem I have/had to solve. Fun.
                int numSprings = 2 * detailX * detailY + detailX * py + detailY * px;

                int numSurfaces = 2 * (detailX + detailY);

                _alloc_sb(sb, numPoints, numSurfaces, numSprings);

                // Do points
                int pt = 0;
                float dx = scale.x / detailX;
                float dy = scale.y / detailX;
                Vector2 average = Vector2Scale(scale, -0.5);
                for (int x = 0; x < px; x++) {
                        float nx = dx * x;
                        for (int y = 0; y < py; y++) {
                                float ny = dy * y;
                                Vector2 newPt = {nx, ny};
                                sb->pointPos[pt] = Vector2Add(newPt, center);
                                sb->pointVel[pt] = Vector2Zero();
                                sb->shape[pt] = Vector2Add(newPt, average);
                                pt++;
                        }
                }
                // Do vertical struts
                int spring = 0;
                for (int x = 0; x < px; x++) {
                        int pmy = x * py;
                        for (int y = 0; y < detailY; y++) {
                                int p = pmy + y;
                                sb->springA[spring] = p;
                                sb->springB[spring] = p + 1;
                                sb->lengths[spring] = dy;
                                spring++;
                        }
                }
                // Do horizontal struts
                for (int x = 0; x < detailX; x++) {
                        // This is why we don't swap the for loops
                        int pmy = x * py;
                        for (int y = 0; y < py; y++) {
                                int p = pmy + y;
                                sb->springA[spring] = p;
                                sb->springB[spring] = p + px;
                                sb->lengths[spring] = dx;
                                spring++;
                        }
                }
                // Diagonal struts
                // Actually does two points at once;
                // Think of it as traversing through the squares between the
                // points, marked by the point to its top-left (remember in
                // this engine +y is down)
                float diagDst = hypotf(dx, dy);
                for (int x = 0; x < detailX; x++) {
                        int pmy = x * py;
                        for (int y = 0; y < detailY; y++) {
                                int p = pmy + y;
                                // Down-right strut
                                sb->springA[spring] = p;
                                sb->springB[spring] = p + px + 1;
                                sb->lengths[spring] = diagDst;
                                spring++;
                                // Up-left struct
                                sb->springA[spring] = p + 1;
                                sb->springB[spring] = p + px;
                                sb->lengths[spring] = diagDst;
                                spring++;
                        }
                }
                // Do surfaces
                int surface = 0;
                // Top and Bottom
                for (int x = 0; x < detailX; x++) {
                        // top
                        int p = x * px;
                        sb->surfaceA[surface] = p;
                        sb->surfaceB[surface] = p + px;
                        surface++;
                        // bottom
                        int bp = p + px - 1;
                        sb->surfaceA[surface] = bp;
                        sb->surfaceB[surface] = bp + px;
                        surface++;
                }
                // Left and Right
                for (int y = 0; y < detailY; y++) {
                        // Left
                        sb->surfaceA[surface] = y;
                        sb->surfaceB[surface] = y + 1;
                        surface++;
                        // Right
                        int p = y + detailX * px;
                        sb->surfaceA[surface] = p;
                        sb->surfaceB[surface] = p + 1;
                        surface++;
                }

        } else {
                int numPoints = 2 * (detailX + detailY);

                _alloc_sb(sb, numPoints, numPoints, numPoints);

                int pt = 0;
                float dx = scale.x / detailX;
                float dy = scale.y / detailX;
                Vector2 tracker = {scale.x * 0.5, scale.y * 0.5};
                // TODO: Try out other idea that directly subdivides surfaces, rather than traverses it
                // Right side Upwards
                for (int y = 0; y < detailY; y++) {
                        sb->pointPos[pt] = Vector2Add(tracker, center);
                        sb->pointVel[pt] = Vector2Zero();
                        sb->shape[pt] = tracker;
                        sb->surfaceA[pt] = pt;
                        sb->surfaceB[pt] = pt + 1;
                        sb->springA[pt] = pt;
                        sb->springB[pt] = pt + 1;
                        sb->lengths[pt] = dy;
                        pt++;
                        tracker.y -= dy;
                }
                // Top side Leftwards
                for (int x = 0; x < detailX; x++) {
                        sb->pointPos[pt] = Vector2Add(tracker, center);
                        sb->pointVel[pt] = Vector2Zero();
                        sb->shape[pt] = tracker;
                        sb->surfaceA[pt] = pt;
                        sb->surfaceB[pt] = pt + 1;
                        sb->springA[pt] = pt;
                        sb->springB[pt] = pt + 1;
                        sb->lengths[pt] = dx;
                        pt++;
                        tracker.x -= dx;
                }
                // Right side Downwards
                for (int y = 0; y < detailY; y++) {
                        sb->pointPos[pt] = Vector2Add(tracker, center);
                        sb->pointVel[pt] = Vector2Zero();
                        sb->shape[pt] = tracker;
                        sb->surfaceA[pt] = pt;
                        sb->surfaceB[pt] = pt + 1;
                        sb->springA[pt] = pt;
                        sb->springB[pt] = pt + 1;
                        sb->lengths[pt] = dy;
                        pt++;
                        tracker.y += dy;
                }
                // Bottom side Rightwards
                for (int x = 0; x < detailX; x++) {
                        sb->pointPos[pt] = Vector2Add(tracker, center);
                        sb->pointVel[pt] = Vector2Zero();
                        sb->shape[pt] = tracker;
                        sb->surfaceA[pt] = pt;
                        sb->surfaceB[pt] = pt + 1;
                        sb->springA[pt] = pt;
                        sb->springB[pt] = pt + 1;
                        sb->lengths[pt] = dx;
                        pt++;
                        tracker.x += dx;
                }
                sb->surfaceB[pt - 1] = 0;
                sb->springB[pt - 1] = 0;
        }
}

SoftBody createEmptySoftBody(SoftBodyType type, float mass, float linearDrag, float springStrength, float springDamp, float shapeSpringStrength, float nRT) {
        return (SoftBody){
            .type = type,
            .numPoints = 0,
            .numSurfaces = 0,
            .numSprings = 0,
            .mass = mass,
            .linearDrag = linearDrag,
            .springStrength = springStrength,
            .springDamp = springDamp,
            .shapeSpringStrength = shapeSpringStrength,
            .nRT = nRT,
        };
}

void _alloc_sb(SoftBody *sb, int numPoints, int numSurfaces, int numSprings) {
        sb->numPoints = numPoints;
        sb->pointPos = MemAlloc(sizeof(Vector2) * numPoints);
        sb->pointVel = MemAlloc(sizeof(Vector2) * numPoints);
        sb->shape = MemAlloc(sizeof(Vector2) * numPoints);
        sb->numSurfaces = numSurfaces;
        sb->surfaceA = MemAlloc(sizeof(int) * numSprings);
        sb->surfaceB = MemAlloc(sizeof(int) * numSprings);
        sb->numSprings = numSprings;
        sb->springA = MemAlloc(sizeof(int) * numSprings);
        sb->springB = MemAlloc(sizeof(int) * numSprings);
        sb->lengths = MemAlloc(sizeof(float) * numSprings);
}

void freeSoftbody(SoftBody *toFree) {
        MemFree(toFree->pointPos);
        MemFree(toFree->pointVel);
        MemFree(toFree->shape);
        MemFree(toFree->surfaceA);
        MemFree(toFree->surfaceB);
        MemFree(toFree->springA);
        MemFree(toFree->springB);
        MemFree(toFree->lengths);
        toFree->numPoints = 0;
        toFree->numSprings = 0;
}