#include <core/collision.h>
#include <math.h>
#include <raylib.h>
#include <stdio.h>

int anypoints(SoftBody A, SoftBody B) {
        // ? Idea: optimize this through vertical decomposition
        //! Doesn't match with the *first* surface it sees, just any surface at all
        //* Might thus be easier to actually do the counting thing
        for (int i = 0; i < A.numPoints; i++) {
                Vector2 point = A.pointPos[i];
                int intersects = 0;

                for (int surf = 0; surf < B.numSurfaces; surf++) {
                        Vector2 p1 = B.pointPos[B.surfaceA[surf]];
                        Vector2 p2 = B.pointPos[B.surfaceB[surf]];

                        float min = fminf(p1.y, p2.y);
                        float max = fmaxf(p1.y, p2.y);

                        if (point.y < min || point.y > max)
                                continue; // Line isn't on the same axis as the point

                        float dx = p2.x - p1.x;
                        float dy = p2.y - p1.y;
                        float cross = (point.x - p1.x) * dy - dx * (point.y - p1.y);

                        if (cross * dy > 0)
                                continue;

                        intersects++;
                }
                if (intersects % 2)
                        return i;
        }
        return -1;
}

void nearestSurface(SoftBody A, int a_i, SoftBody B, int *nearestSurf, float *nearestDist, float *edge_t, Vector2 *nearestPoint) {
        Vector2 point = A.pointPos[a_i];

        *nearestDist = FP_INFINITE;
        for (int surf = 0; surf < B.numSurfaces; surf++) {
                Vector2 p1 = B.pointPos[B.surfaceA[surf]];
                Vector2 p2 = B.pointPos[B.surfaceB[surf]];

                Vector2 diff = Vector2Subtract(p2, p1);

                float l2 = Vector2LengthSqr(diff);
                Vector2 projection;
                // Since we're assuming a closed polygon, we can actually
                // make some assumptions. We don't ever need to clamp,
                // we can just continue, because since it's inside a polygon,
                // we know that it always is closest to an actual point on the segments
                if (l2 == 0.0)
                        continue;

                float t = Vector2DotProduct(Vector2Subtract(point, p1), diff) / l2;

                if (t < 0.f || t > 1.f)
                        continue;

                projection = Vector2Add(p1, Vector2Scale(diff, t)); // Projection falls on the segment

                float dist = Vector2Distance(point, projection);
                if (dist < *nearestDist) {
                        *nearestSurf = surf;
                        *nearestPoint = projection;
                        *nearestDist = dist;
                        *edge_t = t;
                }
        }
}

CollisionData _internalCheckCollision(SoftBody A, SoftBody B) {
        int p = anypoints(A, B);
        if (p == -1) {
                return (CollisionData){.collided = false};
        }

        CollisionData data = (CollisionData){.invert = false, .collided = true, .point = p};
        nearestSurface(A, p, B, &data.edge, &data.distance, &data.edge_t, &data.nearest);

        return data;
}

CollisionData checkCollision(SoftBody A, SoftBody B) {
        // Top-down refinement
        // First we devise an algorithm for detecing collision
        // and effeciently finding the colliding points

        // Check bounding boxes
        if (
            A.bounds.max.x < B.bounds.min.x ||
            A.bounds.max.y < B.bounds.min.y ||
            A.bounds.min.x > B.bounds.max.x ||
            A.bounds.min.y > B.bounds.max.y) {
                return (CollisionData){.collided = false};
        }

        CollisionData a = _internalCheckCollision(A, B);
        if (!a.collided) {
                a = _internalCheckCollision(B, A);
                a.invert = true;
        }

        return a;
}

// TODO: Actually populate these, maybe make it some compile-time autogenerated thing from a config file
// TODO: Implement friction
float getFriction(SoftBodyMaterial A, SoftBodyMaterial B) {
        return 1.0;
}

// Assumes the collision is actually collided and already un-inverted
void _handleCollision_internal(SoftBody A, SoftBody B, CollisionData data, SoftBodyMaterial matA, SoftBodyMaterial matB, float dt) {
        // Based on the ideas from JellyCar Worlds collision

        // First, shift the positions so the shapes are no longer intersecting

        // When solving for the math, I boiled down a solution into this:
        // Let...
        //   A and B be the points of the edge/surface,
        //   v be the point inside of it,
        //   u be the `edge_t`,
        //   m1 be the mass of points of the surface = `B.mass`
        //   m2 be the mass of the point inside = `A.mass`
        // Then...
        //   o = A + (B - A)u - v
        // The solution is given like so, where the subscript 1 is the new positions for the points:
        //   A1 = A - ao
        //   B1 = B - bo
        //   v1 = v + po
        // Where a, b, and p are now the unknowns we must solve for
        // We thus get these constraints:
        //   1) m1*a + m1*b - m2*p = 0  // Signifies the masses
        //   2) b/a = u/(1-u) // Signifies the "closeness" to either edge of the surface
        //   3) A1, B1, and v are colinear
        // It was *very* difficult to try to coax that last one into some linear equation, but I did, so
        // we can represent our solution very simply as the following augmented matrix:
        // [   m1     m1 -m2 |  0  ]
        // | u/(1-u) -1   0  |  0  |
        // [   1-u    u   1  |  1  ]
        // And thus we can solve using Cramer's rule (since in this case N=3 it's the best (I think))
        // In the code we rename A/B to p1/p2

        int bsa = B.surfaceA[data.edge];
        int bsb = B.surfaceB[data.edge];

        Vector2 p1 = B.pointPos[bsa],
                p2 = B.pointPos[bsb],
                v = A.pointPos[data.point];
        float u = data.edge_t, m1 = B.mass, m2 = A.mass;

        // o = p1 + (p2 - p1)*u - v;
        Vector2 o = Vector2Subtract(Vector2Add(p1, Vector2Scale(Vector2Subtract(p2, p1), u)), v);

        // Now for a very hacked-together Cramer's rule
        float ud1mu = u / (1 - u);
        float det = -m1 - (m1 * ud1mu) - m2 * (ud1mu * u + 1.f - u);

        float a = -m2 / det;                // det(M_1) very nicely simplifies
        float b = ud1mu * a;                // We find this directly from the second
        float p = (a + b) * m1 * A.invMass; // And this directly from the first

        // And our solutions are:
        Vector2 A1 = Vector2Subtract(p1, Vector2Scale(o, a));
        Vector2 B1 = Vector2Subtract(p2, Vector2Scale(o, b));
        Vector2 v1 = Vector2Add(v, Vector2Scale(o, p));

        B.pointPos[bsa] = A1;
        B.pointPos[bsb] = B1;
        A.pointPos[data.point] = v1;

        // printf("We got here...\n");
        // debugString = "Ok now this is weird";
        // debugStr = TextFormat("%i   %f, %f", bsa, o.x, o.y);

        // Since we are literally simulating the elasticity of objects,
        // we don't have to worry about coeffs of restitution or anything
        // We just have to choose between making collisions either
        // completely elastic or inelasitic
        // * We're trying the inelastic first

        // Again, we have some linear equations as constraints
        // Conservation of momentum:
        // 2 * m1 * v_b.x + m2 * v_a.x = (2 * m1 + m2) * v_f.x
        // 2 * m1 * v_b.y + m2 * v_a.y = (2 * m1 + m2) * v_f.y
        //
        // Thus we get:
        // v_f.x = (2 * m1 * v_b.x + m2 * v_a.x) / (2 * m1 + m2)
        // v_f.y = (2 * m1 * v_b.y + m2 * v_a.y) / (2 * m1 + m2)
        //
        // And we can properly re-distribute v_b by assuming that frame of reference
        // So now we basically have v_b = 0, so (rearranged)
        // v_f.x = v_rel.x * m2 / (2 * m1 + m2)
        // v_f.y = v_rel.y * m2 / (2 * m1 + m2)

        Vector2 b_avg_vel = Vector2Add(
            Vector2Scale(B.pointVel[bsa], 1 - u),
            Vector2Scale(B.pointVel[bsb], u));

        Vector2 relative_vel = Vector2Subtract(b_avg_vel, A.pointVel[data.point]);

        // Most importantly check if they're already moving apart to skip
        Vector2 normal = (Vector2){A1.y - B1.y, B1.x - A1.x};
        if (Vector2DotProduct(relative_vel, normal) > 0)
                return;

        Vector2 final_vel = Vector2Scale(relative_vel, m2 / (2 * m1 + m2));

        A.pointVel[data.point] = Vector2Add(final_vel, b_avg_vel);

        // Redistribute B velocities
        B.pointVel[bsa] = Vector2Subtract(b_avg_vel, Vector2Scale(final_vel, 1 - u));
        B.pointVel[bsb] = Vector2Subtract(b_avg_vel, Vector2Scale(final_vel, u));
}

void handleCollision(SoftBody A, SoftBody B, CollisionData data, SoftBodyMaterial matA, SoftBodyMaterial matB, float dt) {
        if (data.invert)
                _handleCollision_internal(B, A, data, matA, matB, dt);
        else
                _handleCollision_internal(A, B, data, matA, matB, dt);
}
