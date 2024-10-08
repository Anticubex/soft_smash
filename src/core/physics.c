#include <assert.h>
#include <bettermath.h>
#include <core/physics.h>
#include <stddef.h>
#include <string.h>

void update_SoftBody(SoftBody *sb, WorldValues worldValues, float dt) {
        // Now for the behemoth

        // Use arena allocation because I had a suspicious feeling
        // That some sort of memory leak was happening. Also arena alloc is cool.
        int n = sb->numPoints;

        Vector2 *arenaAlloc = MemAlloc(sizeof(Vector2) * n * 9);

        Vector2 *k1, *k2, *k3, *k4, *final;
        SBPoints ogpoints, newpoints;

        ogpoints = (SBPoints){
            .num = n,
            .pos = arenaAlloc + 0 * n,
            .vel = arenaAlloc + 1 * n,
        };
        memcpy(ogpoints.pos, sb->pointPos, sizeof(Vector2) * n);
        memcpy(ogpoints.vel, sb->pointVel, sizeof(Vector2) * n);
        k1 = arenaAlloc + 2 * n;
        calcForces(k1, *sb, ogpoints, worldValues);

        newpoints = (SBPoints){
            .num = n,
            .pos = arenaAlloc + 3 * n,
            .vel = arenaAlloc + 4 * n,
        };

        projectSB(&newpoints, ogpoints, k1, dt * 0.5, sb->invMass);
        k2 = arenaAlloc + 5 * n;
        calcForces(k2, *sb, newpoints, worldValues);

        projectSB(&newpoints, ogpoints, k2, dt * 0.5, sb->invMass);
        k3 = arenaAlloc + 6 * n;
        calcForces(k3, *sb, newpoints, worldValues);

        projectSB(&newpoints, ogpoints, k3, dt, sb->invMass);
        k4 = arenaAlloc + 7 * n;
        calcForces(k4, *sb, newpoints, worldValues);
        final = arenaAlloc + 8 * n;
        sumForces(n, final, k1, 0.16666f);
        sumForces(n, final, k2, 0.33333f);
        sumForces(n, final, k3, 0.33333f);
        sumForces(n, final, k4, 0.16666f);

        projectSB(&newpoints, ogpoints, final, dt, sb->invMass);
        apply_SBPoints(sb, newpoints);

        MemFree(arenaAlloc);

        SBPos newPos = calcShape(*sb, newpoints);
        sb->shapePosition = newPos.position;
        sb->shapeRotation = newPos.rotation;

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
}

void calcForces(Vector2 *forces, SoftBody sb, SBPoints points, WorldValues worldValues) {
        if (sb.type & SoftBodyType_Springs) {
                calcForce_springs(forces, sb, points, worldValues);
        }
        if (sb.type & SoftBodyType_Shape) {
                calcForce_shape(forces, sb, points, worldValues);
        }
        if (sb.type & SoftBodyType_Pressure) {
                calcForce_pressure(forces, sb, points, worldValues);
        }
        // Now for gravity
        for (int i = 0; i < points.num; i++) {
                forces[i] = Vector2Add(forces[i], Vector2Scale(worldValues.gravity, sb.mass));
        }
        calcForce_drag(forces, sb, points, worldValues);
}

SBPos calcShape(SoftBody sb, SBPoints points) { // Recalculate frame center and rotation
        Vector2 avg_pos = {0, 0};
        for (int i = 0; i < sb.numPoints; i++) {
                avg_pos.x += points.pos[i].x;
                avg_pos.y += points.pos[i].y;
        }
        avg_pos = Vector2Scale(avg_pos, 1.f / sb.numPoints);
        float avg_rotation = 0;
        for (int i = 0; i < sb.numPoints; i++) {
                // TODO: Fix this, as it averages not angles but rotations, but also that might be what we need, idk, put some more analysis/debugging into this line
                avg_rotation += Vector2Angle(sb.shape[i], Vector2Subtract(points.pos[i], avg_pos));
        }
        avg_rotation /= sb.numPoints;

        return (SBPos){
            .position = avg_pos,
            .rotation = avg_rotation,
        };
}

void calcForce_springs(Vector2 *forces, SoftBody sb, SBPoints points, WorldValues worldValues) {
        for (int i = 0; i < sb.numSprings; i++) {
                int a_idx = sb.springA[i];
                int b_idx = sb.springB[i];
                Vector2 a_pos = points.pos[a_idx];
                Vector2 b_pos = points.pos[b_idx];
                Vector2 a_vel = points.vel[a_idx];
                Vector2 b_vel = points.vel[b_idx];
                Vector2 diff = Vector2Subtract(a_pos, b_pos);

                // You'd think this would be the easiest, but no, I spent several hours debugging
                // this to find out that it was wrong in 20 different ways, most notable of which
                // being that spring damping was more like spring lightspeed-accelerator.
                // This was what spurred me into doing RK4.

                // a
                // ^
                // b

                // a moving into b
                //
                // (0,0)    (1,0)
                //   a   >>   b
                // (1,1)    (0,2)
                //
                // b' - a' = -1, 1
                // diff = b - a = (-1, 0) = <-
                // springLen = 2
                // diffLen = 1
                // damp = (b' - a') * diffNorm = 1
                // x = springLen - diffLen = 2 - 1 = 1
                // diff * x = (-1, 0) correct
                // diff * damp = (-1, 0) correct

                // Calculate force
                // f = -kx - cx'

                float length = Vector2Length(diff);
                Vector2 diffNorm = Vector2Scale(diff, 1. / length);
                float x = sb.lengths[i] - length;

                float springForce = sb.springStrength * x;
                float dampForce = sb.springDamp * Vector2DotProduct(Vector2Subtract(b_vel, a_vel), diffNorm);

                float f = springForce + dampForce;

                forces[a_idx] = Vector2Add(forces[a_idx], Vector2Scale(diffNorm, f));
                forces[b_idx] = Vector2Add(forces[b_idx], Vector2Scale(diffNorm, -f));
        }
}

void calcForce_shape(Vector2 *forces, SoftBody sb, SBPoints points, WorldValues worldValues) {

        SBPos pos = calcShape(sb, points);
        Matrix shapeMatrix = MatrixIdentity();
        float sinA = sinf(pos.rotation);
        float cosA = cosf(pos.rotation);
        shapeMatrix.m0 = cosA;
        shapeMatrix.m1 = sinA;
        shapeMatrix.m4 = -sinA;
        shapeMatrix.m5 = cosA;

        for (int i = 0; i < sb.numPoints; i++) {
                // Calculate distance
                Vector2 shape_pos = Vector2Add(Vector2Transform(sb.shape[i], shapeMatrix), pos.position);
                Vector2 diff = Vector2Subtract(shape_pos, points.pos[i]);
                forces[i] = Vector2Add(forces[i], Vector2Scale(diff, sb.shapeSpringStrength));
        }
}

void calcForce_pressure(Vector2 *forces, SoftBody sb, SBPoints points, WorldValues worldValues) {
        // Calculate Volume
        // Use shoelace formula, summing matrix determinants along the edges
        float V = 0.f;
        for (int i = 0; i < sb.numSurfaces; i++) {
                Vector2 a = points.pos[sb.surfaceA[i]];
                Vector2 b = points.pos[sb.surfaceB[i]];
                V += a.x * b.y - a.y * b.x;
        }
        V *= 0.5f;
        float P = sb.nRT / V;

        // Apply Pressure
        for (int i = 0; i < sb.numSurfaces; i++) {
                int a_idx = sb.surfaceA[i];
                int b_idx = sb.surfaceB[i];
                Vector2 a = points.pos[a_idx];
                Vector2 b = points.pos[b_idx];
                Vector2 diff = Vector2Subtract(a, b);

                // Multiplying P by the difference saves us having to calculate the length
                // This is where the CCW winding comes in

                Vector2 normal = {-diff.y * P, diff.x * P};

                forces[a_idx] = Vector2Add(forces[a_idx], normal);
                forces[b_idx] = Vector2Add(forces[b_idx], normal);
        }
}

void calcForce_drag(Vector2 *forces, SoftBody sb, SBPoints points, WorldValues worldValues) {
        // Does apply a sort of drag to surface facing away from the direction
        // Of motion because that's more mathematically elegant and so easier
        // to code. One thing to add later is to thing about the torque of
        // each surface segment. If one point/side is moving faster than the
        // other, more of the force should be applied to that side
        // ? Hmm. Think this through more later
        for (int i = 0; i < sb.numSurfaces; i++) {
                // Get dot product and filter negative ones out
                int a = sb.surfaceA[i];
                int b = sb.surfaceB[i];
                Vector2 surface = Vector2Subtract(points.pos[a], points.pos[b]);
                Vector2 surface_outv = (Vector2){-surface.y, surface.x};
                float isl = 1.0f / Vector2Length(surface); // TODO: consider fast inv sqrt
                // Thinking about this, this actually works well with the projective nature of the dot product
                // Just due to the algebra
                // F_d = 0.5 * density of fluid * relative speed^2 * coeff of drag * cross-sectional area
                Vector2 velocity = Vector2Scale(Vector2Add(points.vel[a], points.vel[b]), 0.5f);
                float v2 = Vector2LengthSqr(velocity);
                float v = sqrtf(v2);
                // The algebra works *flawlessly* so we don't need to normalize anything, just divide the lengths
                // I mean, the code doesn't, obviously, but the algebra is CLEAN, ELEGANT

                float dot = Vector2DotProduct(surface_outv, velocity);
                if (dot <= 0.f)
                        continue;

                float F_D = 0.5 * worldValues.airPressure * sb.linearDrag * dot * v * isl;

                forces[a] = Vector2Add(forces[a], Vector2Scale(surface_outv, -F_D));
                forces[b] = Vector2Add(forces[b], Vector2Scale(surface_outv, -F_D));
        }
}

void projectSB(SBPoints *dest, SBPoints src, Vector2 *forces, float dt, float invMass) {
        for (int i = 0; i < src.num; i++) {
                Vector2 newVel = dest->vel[i] = Vector2Add(src.vel[i], Vector2Scale(forces[i], dt * invMass));
                dest->pos[i] = Vector2Add(src.pos[i], Vector2Scale(newVel, dt));
        }
}

void SBPoint_addForce(SoftBody *sb, int i, Vector2 force, float dt) {
        sb->pointVel[i] = Vector2Add(sb->pointVel[i], Vector2Scale(force, dt));
}

Vector2 *alloc_forces(int num) {
        return MemAlloc(sizeof(Vector2) * num);
}

SBPoints rip_SBPoints(SoftBody sb) {
        int num = sb.numPoints;
        unsigned int size = sizeof(Vector2) * num;
        Vector2 *pos = MemAlloc(size);
        Vector2 *vel = MemAlloc(size);
        memcpy(pos, sb.pointPos, size);
        memcpy(vel, sb.pointVel, size);
        return (SBPoints){
            .num = sb.numPoints,
            .pos = pos,
            .vel = vel,
        };
}

void apply_SBPoints(SoftBody *sb, SBPoints points) {
        // Slower but most definitely safer
        memcpy(sb->pointPos, points.pos, sizeof(Vector2) * points.num);
        memcpy(sb->pointVel, points.vel, sizeof(Vector2) * points.num);
        // // God if this leads to bugs will it be hell to debug
        // So yeah it immediately led to bugs
        // MemFree(sb->pointPos);
        // MemFree(sb->pointVel);
        // sb->pointPos = points.pos;
        // sb->pointVel = points.vel;
}

void alloc_SBPoints(SBPoints *points, int num) {
        points->num = num;
        points->pos = MemAlloc(sizeof(Vector2) * num);
        points->vel = MemAlloc(sizeof(Vector2) * num);
}

void free_SBPoints(SBPoints *points) {
        points->num = 0;
        MemFree(points->pos);
        MemFree(points->vel);
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

        _center_sb_shape(sb);
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
                                sb->springB[spring] = p + py;
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
                                sb->springB[spring] = p + py + 1;
                                sb->lengths[spring] = diagDst;
                                spring++;
                                // Up-left struct
                                sb->springA[spring] = p + 1;
                                sb->springB[spring] = p + py;
                                sb->lengths[spring] = diagDst;
                                spring++;
                        }
                }
                // Do surfaces, remember to keep orientation
                int surface = 0;

                // Top rightwards
                for (int x = 0; x < detailX; x++) {
                        sb->surfaceA[surface] = x * py;
                        sb->surfaceB[surface] = (x + 1) * py;
                        surface++;
                }
                // Right downwards
                for (int y = 0; y < detailY; y++) {
                        sb->surfaceA[surface] = detailX * py + y;
                        sb->surfaceB[surface] = detailX * py + y + 1;
                        surface++;
                }
                // Bottom leftwards
                for (int x = 0; x < detailX; x++) {
                        sb->surfaceA[surface] = px * py - 1 - x * py;
                        sb->surfaceB[surface] = px * py - 1 - (x + 1) * py;
                        surface++;
                }
                // Left upwards
                for (int y = 0; y < detailY; y++) {
                        sb->surfaceA[surface] = detailY - y;
                        sb->surfaceB[surface] = detailY - y - 1;
                        surface++;
                }

        } else {
                int numPoints = 2 * (detailX + detailY);

                _alloc_sb(sb, numPoints, numPoints, numPoints);

                int pt = 0;
                float dx = scale.x / detailX;
                float dy = scale.y / detailX;
                Vector2 tracker = {0, 0};
                // TODO: Try out other idea that directly subdivides surfaces, rather than traverses it
                // Top side Rightwards
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
                // Bottom side Leftwards
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
                sb->surfaceB[pt - 1] = 0;
                sb->springB[pt - 1] = 0;
        }

        _center_sb_shape(sb);
}

SoftBody createEmptySoftBody(SoftBodyType type, float mass, float linearDrag, float springStrength, float springDamp, float shapeSpringStrength, float nRT) {
        return (SoftBody){
            .type = type,
            .numPoints = 0,
            .numSurfaces = 0,
            .numSprings = 0,
            .mass = mass,
            .invMass = 1.f / mass,
            .linearDrag = linearDrag,
            .springStrength = springStrength,
            .springDamp = springDamp,
            .shapeSpringStrength = shapeSpringStrength,
            .nRT = nRT,
        };
}

void _center_sb_shape(SoftBody *sb) {
        Vector2 avg_pos = {0, 0};
        for (int i = 0; i < sb->numPoints; i++) {
                avg_pos.x += sb->shape[i].x;
                avg_pos.y += sb->shape[i].y;
        }
        avg_pos = Vector2Scale(avg_pos, 1.f / sb->numPoints);
        for (int i = 0; i < sb->numPoints; i++) {
                sb->shape[i].x -= avg_pos.x;
                sb->shape[i].y -= avg_pos.y;
        }
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

void sumForces(int num, Vector2 *forces, Vector2 *other, float multiplier) {
        for (int i = 0; i < num; i++) {
                forces[i] = Vector2Add(forces[i], Vector2Scale(other[i], multiplier));
        }
}
