#include "core/physics.h"
#include "physics.h"
#include <math.h>
#include <stdlib.h>

void update_PointBody(PointBody pb, WorldValues worldValues, float dt) {
        pb.position = Vector2Add(pb.position, Vector2Scale(pb.velocity, dt));

        pb.velocity = Vector2Add(pb.velocity, Vector2Scale(worldValues.gravity, dt));
        // Apply drag
        float drag_coeff = (pb.linearDrag < 0) ? worldValues.linearDrag : pb.linearDrag;
        if (!FloatEquals(drag_coeff, 0.f)) {
                pb.velocity = Vector2Scale(
                    pb.velocity,
                    1. - dt * (0.5 *
                               worldValues.airDensity *
                               Vector2Length(pb.velocity) *
                               drag_coeff *
                               pb.invMass));
        }
}

void update_SoftBody(SoftBody sb, WorldValues worldValues, float dt) {
        // Now for the behemoth

        // Update every point
        for (int i = 0; i < sb.num_points; i++) {
                update_PointBody(sb.points[i], worldValues, dt);
        }
        // Update bounding box
        float minx = sb.points[0].position.x;
        float maxx = minx;
        float miny = sb.points[0].position.y;
        float maxy = miny;
        for (int i = 1; i < sb.num_points; i++) {
                Vector2 pos = sb.points[i].position;
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
                for (int i = 0; i < sb.num_points; i++) {
                        avg_pos.x += sb.points[i].position.x;
                        avg_pos.y += sb.points[i].position.y;
                }
                avg_pos = Vector2Scale(avg_pos, 1 / sb.num_points);
                float avg_rotation = 0;
                for (int i = 0; i < sb.num_points; i++) {
                        avg_rotation += Vector2Angle(Vector2Add(sb.shape[i], avg_pos), sb.points[i].position);
                }
                avg_rotation /= sb.num_points;

                Matrix shapeMatrix = MatrixIdentity();
                float sinA = sinf(avg_rotation);
                float cosA = cosf(avg_rotation);
                shapeMatrix.m0 = cosA;
                shapeMatrix.m1 = sinA;
                shapeMatrix.m4 = -sinA;
                shapeMatrix.m5 = cosA;

                for (int i = 0; i < sb.num_points; i++) {
                        // Calculate distance
                        Vector2 shape_pos = Vector2Add(Vector2Transform(sb.shape[i], shapeMatrix), avg_pos);
                        Vector2 diff = Vector2Subtract(shape_pos, sb.points[i].position);
                        float f = Vector2Length(diff) * sb.shapeSpringStrength;
                        PointBody_applyForce(&sb.points[i], Vector2Scale(Vector2Normalize(diff), f), dt);
                }
        }
        // Note: trying to do pressure on a softbody without at least 2 points leads to a segfault.
        // Hmm. Blame it on the user.
        if (sb.type & SoftBodyType_Pressure) {
                // Calculate Volume
                float V =
                    sb.points[0].position.y * (sb.points[sb.num_points - 1].position.x - sb.points[1].position.x) +
                    sb.points[sb.num_points - 1].position.y * (sb.points[sb.num_points - 2].position.x - sb.points[0].position.x);
                // Do edge cases
                for (int i = 1; i < sb.num_points - 1; i++) {
                        V += sb.points[i].position.y * (sb.points[i - 1].position.x - sb.points[i + 1].position.x);
                }
                V *= 0.5;
                // Use gas law for pressure
                float P = sb.nRT / V;

                // Apply Pressure
                for (int i = 0; i < sb.num_points; i++) {
                        PointBody a = sb.points[i];
                        PointBody b = sb.points[(i + 1) == sb.num_points ? 0 : i + 1];
                        Vector2 diff = Vector2Subtract(a.position, b.position);

                        // Multiplying P by the difference saves us having to calculate the length
                        // This is where the CCW winding comes in

                        Vector2 normal = {diff.y * P, -diff.x * P};

                        PointBody_applyForce(&a, normal, dt);
                        PointBody_applyForce(&b, normal, dt);
                }
        }

        // This one's gonna be a marathon to compute. Use sparingly.
        if (sb.type & SoftBodyType_SelfColliding) {
                for (int i = 0; i < sb.num_points - 1; i++) {
                        for (int j = i++; j < sb.num_points; j++) {
                                PointBody a = sb.points[i];
                                PointBody b = sb.points[j];
                                Vector2 diff = Vector2Subtract(a.position, b.position);

                                float length = Vector2LengthSqr(diff);

                                if (length > sb.radii * sb.radii)
                                        continue;

                                Vector2 midpoint = Vector2Scale(Vector2Add(a.position, b.position), 0.5);
                                Vector2 newHalfDiff = Vector2Scale(Vector2Normalize(diff), sb.radii);

                                a.position = Vector2Add(midpoint, newHalfDiff);
                                b.position = Vector2Subtract(midpoint, newHalfDiff);
                        }
                }
        }

        // Now for your basic spring force
        for (int i = 0; i < sb.num_springs; i++) {
                PointBody a = sb.points[sb.springA[i]];
                PointBody b = sb.points[sb.springB[i]];
                Vector2 diff = Vector2Subtract(a.position, b.position);
                // b -> a

                // Calculate force
                // f = -kx - cx'

                float length = Vector2Length(diff);
                Vector2 diffNorm = Vector2Scale(diff, 1. / length);
                float x = length - sb.lengths[i];

                float f = -sb.springStrength * x -
                          sb.springDamp * Vector2DotProduct(Vector2Subtract(a.velocity, b.velocity), diffNorm);

                PointBody_applyForce(&a, Vector2Scale(diffNorm, f), dt);
                PointBody_applyForce(&b, Vector2Scale(diffNorm, -f), dt);
        }

        // TODO : Softbody drag, using area and stuff
}

void PointBody_applyForce(PointBody *pb, Vector2 force, float dt) {
        pb->velocity = Vector2Add(pb->velocity, Vector2Scale(force, dt * pb->invMass));
}

Vector2 rotateQuarterTurn(Vector2 v) {
        return (Vector2){-v.y, v.x};
}
