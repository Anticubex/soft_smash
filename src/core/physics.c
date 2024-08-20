#include "core/physics.h"
#include "physics.h"
#include <bettermath.h>

void SBPoint_addForce(SoftBody sb, int i, Vector2 force, float dt);

void update_SoftBody(SoftBody sb, WorldValues worldValues, float dt) {
        // Now for the behemoth

        // Update every point
        for (int i = 0; i < sb.numPoints; i++) {
                sb.pointPos[i] = Vector2Add(sb.pointPos[i], Vector2Scale(sb.pointVel[i], dt));
        }
        // Update bounding box
        float minx = sb.pointPos[0].x;
        float maxx = minx;
        float miny = sb.pointPos[0].y;
        float maxy = miny;
        for (int i = 1; i < sb.numPoints; i++) {
                Vector2 pos = sb.pointPos[i];
                minx = pos.x < minx ? pos.x : minx;
                maxx = pos.x > maxx ? pos.x : maxx;
                miny = pos.y < miny ? pos.y : miny;
                maxy = pos.y > maxy ? pos.y : maxy;
        }
        sb.bounds.min = (Vector2){minx, miny};
        sb.bounds.max = (Vector2){maxx, maxy};

        if (sb.type & SoftBodyType_Shape) {
                // Recalculate frame center and rotation
                Vector2 avg_pos = {0, 0};
                for (int i = 0; i < sb.numPoints; i++) {
                        avg_pos.x += sb.pointPos[i].x;
                        avg_pos.y += sb.pointPos[i].y;
                }
                avg_pos = Vector2Scale(avg_pos, 1 / sb.numPoints);
                float avg_rotation = 0;
                for (int i = 0; i < sb.numPoints; i++) {
                        avg_rotation += Vector2Angle(Vector2Add(sb.shape[i], avg_pos), sb.pointPos[i]);
                }
                avg_rotation /= sb.numPoints;

                Matrix shapeMatrix = MatrixIdentity();
                float sinA = sinf(avg_rotation);
                float cosA = cosf(avg_rotation);
                shapeMatrix.m0 = cosA;
                shapeMatrix.m1 = sinA;
                shapeMatrix.m4 = -sinA;
                shapeMatrix.m5 = cosA;

                for (int i = 0; i < sb.numPoints; i++) {
                        // Calculate distance
                        Vector2 shape_pos = Vector2Add(Vector2Transform(sb.shape[i], shapeMatrix), avg_pos);
                        Vector2 diff = Vector2Subtract(shape_pos, sb.pointPos[i]);
                        float f = Vector2Length(diff) * sb.shapeSpringStrength;
                        PointBody_applyForce(&sb.pointPos[i], Vector2Scale(Vector2Normalize(diff), f), dt);
                }
        }
        // Note: trying to do pressure on a softbody without at least 2 points leads to a segfault.
        // Hmm. Blame it on the user.
        if (sb.type & SoftBodyType_Pressure) {
                // Calculate Volume
                float V =
                    sb.pointPos[0].y * (sb.pointPos[sb.numPoints - 1].x - sb.pointPos[1].x) +
                    sb.pointPos[sb.numPoints - 1].y * (sb.pointPos[sb.numPoints - 2].x - sb.pointPos[0].x);
                // Do edge cases
                for (int i = 1; i < sb.numPoints - 1; i++) {
                        V += sb.pointPos[i].y * (sb.pointPos[i - 1].x - sb.pointPos[i + 1].x);
                }
                V *= 0.5;
                // Use gas law for pressure
                float P = sb.nRT / V;

                // Apply Pressure
                for (int i = 0; i < sb.numPoints; i++) {
                        Vector2 a = sb.pointPos[i];
                        Vector2 b = sb.pointPos[(i + 1) % sb.numPoints];
                        Vector2 diff = Vector2Subtract(a, b);

                        // Multiplying P by the difference saves us having to calculate the length
                        // This is where the CCW winding comes in

                        Vector2 normal = {diff.y * P, -diff.x * P};

                        SBPoint_addForce(sb, i, normal, dt);
                        SBPoint_addForce(sb, (i + 1) % sb.numPoints, normal, dt);
                }
        }

        // Now for your basic spring force
        for (int i = 0; i < sb.numSprings; i++) {
                int a_idx = sb.springA[i];
                int b_idx = sb.springB[i];
                Vector2 a_pos = sb.pointPos[a_idx];
                Vector2 b_pos = sb.pointPos[b_idx];
                Vector2 a_vel = sb.pointVel[a_idx];
                Vector2 b_vel = sb.pointVel[b_idx];
                Vector2 diff = Vector2Subtract(a_pos, b_pos);
                // b -> a

                // Calculate force
                // f = -kx - cx'

                float length = Vector2Length(diff);
                Vector2 diffNorm = Vector2Scale(diff, 1. / length);
                float x = length - sb.lengths[i];

                float f = -sb.springStrength * x -
                          sb.springDamp * Vector2DotProduct(Vector2Subtract(a_vel, b_vel), diffNorm);

                SBPoint_addForce(sb, a_idx, Vector2Scale(diffNorm, f), dt);
                SBPoint_addForce(sb, b_idx, Vector2Scale(diffNorm, -f), dt);
        }

        // TODO : Softbody drag/air resistance, using area and stuff
}

Vector2 rotateQuarterTurn(Vector2 v) {
        return (Vector2){-v.y, v.x};
}
