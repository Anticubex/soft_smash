#include "core/physics.h"
#include "physics.h"
#include <math.h>
#include <stdlib.h>

void update_RigidBody(RigidBody rb, WorldValues worldValues, float dt) {
        rb.transform.position = Vector2Add(rb.transform.position, Vector2Scale(rb.velocity, dt));
        rb.transform.rotation += rb.angularVelocity * dt;

        rb.velocity = Vector2Add(rb.velocity, Vector2Scale(worldValues.gravity, dt));
        // Apply drag
        float drag_coeff = (rb.linearDrag < 0) ? worldValues.linearDrag : rb.linearDrag;
        if (!FloatEquals(drag_coeff, 0.f)) {
                rb.velocity = Vector2Scale(
                    rb.velocity,
                    1. - dt * (0.5 *
                               worldValues.airDensity *
                               Vector2Length(rb.velocity) *
                               drag_coeff *
                               rb.invMass));
        }

        // Apply angular drag
        float angular_drag_coeff = (rb.angularDrag < 0) ? worldValues.angularDrag : rb.angularDrag;
        if (!FloatEquals(angular_drag_coeff, 0.f)) {
                rb.angularVelocity -= rb.angularVelocity * rb.angularVelocity * angular_drag_coeff * rb.invMomentOfInertia * dt;
        }
}

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

RectPoints calculateRectanglePoints(BoxColliderData colliderData, Transform2D transform) {
        Vector2 diff = Vector2Rotate(colliderData.dimensions, colliderData.rotation);
        Vector2 a = applyTransform(transform, Vector2Add(colliderData.center, diff));
        diff = rotateQuarterTurn(diff);
        Vector2 b = applyTransform(transform, Vector2Add(colliderData.center, diff));
        diff = rotateQuarterTurn(diff);
        Vector2 c = applyTransform(transform, Vector2Add(colliderData.center, diff));
        diff = rotateQuarterTurn(diff);
        Vector2 d = applyTransform(transform, Vector2Add(colliderData.center, diff));
        diff = rotateQuarterTurn(diff);
        return (struct RectPoints){a, b, c, d};
}

Collider initPointCollider(Vector2 offset) {
        PointColliderData *data = malloc(sizeof(PointColliderData));
        data->offset = offset;
        return (Collider){
            (BB){0, 0, 0, 0},
            ColliderType_POINT,
            data,
        };
}
Collider initBoxCollider(Vector2 center, Vector2 dimensions, float rotation) {
        BoxColliderData *data = malloc(sizeof(PointColliderData));
        data->center = center;
        data->dimensions = dimensions;
        data->rotation = rotation;
        return (Collider){
            (BB){0, 0, 0, 0},
            ColliderType_BOX,
            data,
        };
}
Collider initCircleCollider(Vector2 offset, float radius) {
        CircleColliderData *data = malloc(sizeof(PointColliderData));
        data->offset = offset;
        data->radius = radius;
        return (Collider){
            (BB){0, 0, 0, 0},
            ColliderType_CIRCLE,
            data,
        };
}
Collider initGroupCollider(int n) {
        GroupColliderData *data = malloc(sizeof(GroupColliderData));
        data->n = n;
        data->types = malloc;
        data->n = n;
        return (Collider){
            (BB){0, 0, 0, 0},
            ColliderType_CIRCLE,
            data,
        };
}

void freeCollider(Collider collider) {
        if (collider.colliderType != ColliderType_GROUP) {
                free(collider.colliderData);
                return;
        }
        // Once again, I regret adding builtin group colliders.
        // But it's too late to back out now.
        typedef struct groupColliderStack {
                GroupColliderData *data;
                struct groupColliderStack *next;
        } groupColliderStack;
        groupColliderStack stack = {(GroupColliderData *)(collider.colliderData), NULL};
        do {
                for (int i = 0; i < stack.data->n; i++) {
                        if (stack.data->types[i] != ColliderType_GROUP) {
                                free(stack.data->colliderDatas[i]);
                                continue;
                        }
                        groupColliderStack *next = alloca(sizeof(groupColliderStack));
                        next->data = (GroupColliderData *)stack.data->colliderDatas[i];
                        next->next = stack.next;
                        stack.next = next;
                }
                free(stack.data->types);
                free(stack.data->colliderDatas);
                free(stack.data);
                stack = *stack.next;
        } while (stack.next != NULL);
}

BB getPointBB(PointColliderData data, Transform2D transform) {
        Vector2 p = applyTransform(transform, data.offset);
        return (BB){p, p};
}
BB getRectBB(BoxColliderData data, Transform2D transform) {
        // Calculate all 4 points
        // Find the minimums and maximums
        struct RectPoints points = calculateRectanglePoints(data, transform);
        // a
        int minx = points.a.x;
        int maxx = points.a.x;
        int miny = points.a.y;
        int maxy = points.a.y;
        // b
        int minx = (minx < points.b.y) ? minx : points.b.x;
        int maxx = (maxx > points.b.y) ? maxx : points.b.x;
        int miny = (minx < points.b.y) ? minx : points.b.y;
        int maxy = (maxy > points.b.y) ? maxy : points.b.y;
        // c
        int minx = (minx < points.c.y) ? minx : points.c.x;
        int maxx = (maxx > points.c.y) ? maxx : points.c.x;
        int miny = (minx < points.c.y) ? minx : points.c.y;
        int maxy = (maxy > points.c.y) ? maxy : points.c.y;
        // d
        int minx = (minx < points.d.y) ? minx : points.d.x;
        int maxx = (maxx > points.d.y) ? maxx : points.d.x;
        int miny = (minx < points.d.y) ? minx : points.d.y;
        int maxy = (maxy > points.d.y) ? maxy : points.d.y;

        return (BB){
            (Vector2){minx, miny}, (Vector2){maxx, maxy}};
}
BB getCircleBB(CircleColliderData data, Transform2D transform) {
        Vector2 realCenter = applyTransform(transform, data.offset);
        return (BB){
            {realCenter.x - data.radius,
             realCenter.x + data.radius},
            {realCenter.y - data.radius,
             realCenter.y + data.radius},
        };
}
BB getGroupBB(GroupColliderData data, Transform2D transform) {
        // Go through each and merge the bounding boxes
        // Can be optimized, but this single code file is already cluttered enough
        // But in the future: could maybe calculate everything without a transform,
        // Then just transform the final BB as a rect and take that as the BB?
        // Points can be merged much more efficiently, as they are literally a single point
        // Would then require making everything unique, but each one could be further optimized
        if (data.n < 1)
                return;
        BB finalbox = findColliderBB((Collider){{0, 0, 0, 0}, data.types[0], data.colliderDatas[0]}, transform);
        for (int i = 1; i < data.n; i++) {
                BB box = findColliderBB((Collider){{0, 0, 0, 0}, data.types[i], data.colliderDatas[i]}, transform);
                finalbox.min.x = finalbox.min.x < box.min.x ? finalbox.min.x : box.min.x;
                finalbox.max.x = finalbox.max.x > box.max.x ? finalbox.max.x : box.max.x;
                finalbox.min.y = finalbox.min.y < box.min.y ? finalbox.min.y : box.min.y;
                finalbox.max.y = finalbox.max.y > box.max.y ? finalbox.max.y : box.max.y;
        }

        return finalbox;
}

BB findColliderBB(Collider collider, Transform2D transform) {
        switch (collider.colliderType) {
        case ColliderType_POINT: {
                return getPointBB(*(PointColliderData *)collider.colliderData, transform);

        } break;
        case ColliderType_BOX: {
                return getRectBB(*(BoxColliderData *)collider.colliderData, transform);
        } break;
        case ColliderType_CIRCLE: {
                return getCircleBB(*(CircleColliderData *)collider.colliderData, transform);
        } break;
        case ColliderType_GROUP: {
                return getGroupBB(*(GroupColliderData *)collider.colliderData, transform);
        } break;
        default:
                return (BB){Vector2Zero(), Vector2Zero()};
                break;
        }
}

bool checkCollision(CollisionData *data, Collider A, Transform2D A_T, Vector2 A_velocity, Collider B, Transform2D B_T, Vector2 B_velocity) {

        // Assumes their bounding boxes have already been updated, with their transforms
        // Check AABB
        BB Ab = A.bounds;
        BB Bb = B.bounds;
        if (!(Ab.max.x >= Bb.min.x && Ab.min.x <= Bb.max.x && Ab.max.y >= Bb.min.y && Ab.min.y <= Bb.max.y))
                return false;

        switch (A.colliderType) {
        case ColliderType_POINT: {
                switch (B.colliderType) {
                case ColliderType_POINT:
                        return checkCollision_PointPoint(
                            data,
                            *(PointColliderData *)A.colliderData,
                            A_T, A_velocity,
                            *(PointColliderData *)B.colliderData,
                            B_T, B_velocity, false);
                        break;
                case ColliderType_BOX:
                        return checkCollision_PointBox(
                            data,
                            *(PointColliderData *)A.colliderData,
                            A_T, A_velocity,
                            *(BoxColliderData *)B.colliderData,
                            B_T, B_velocity, false);
                        break;
                case ColliderType_CIRCLE:
                        return checkCollision_PointCircle(
                            data,
                            *(PointColliderData *)A.colliderData,
                            A_T, A_velocity,
                            *(CircleColliderData *)B.colliderData,
                            B_T, B_velocity, false);
                        break;
                case ColliderType_GROUP:
                        return checkCollision_PointGroup(
                            data,
                            *(PointColliderData *)A.colliderData,
                            A_T, A_velocity,
                            *(GroupColliderData *)B.colliderData,
                            B_T, B_velocity, false);
                        break;
                }
        } break;
        case ColliderType_BOX: {
                switch (B.colliderType) {
                case ColliderType_POINT:
                        return checkCollision_PointBox(
                            data,
                            *(PointColliderData *)B.colliderData,
                            B_T, B_velocity,
                            *(BoxColliderData *)A.colliderData,
                            A_T, A_velocity, true);
                        break;
                case ColliderType_BOX:
                        return checkCollision_BoxBox(
                            data,
                            *(BoxColliderData *)A.colliderData,
                            A_T, A_velocity,
                            *(BoxColliderData *)B.colliderData,
                            B_T, B_velocity, false);
                        break;
                case ColliderType_CIRCLE:
                        return checkCollision_BoxCircle(
                            data,
                            *(BoxColliderData *)A.colliderData,
                            A_T, A_velocity,
                            *(CircleColliderData *)B.colliderData,
                            B_T, B_velocity, false);
                        break;
                case ColliderType_GROUP:
                        return checkCollision_BoxGroup(
                            data,
                            *(BoxColliderData *)A.colliderData,
                            A_T, A_velocity,
                            *(GroupColliderData *)B.colliderData,
                            B_T, B_velocity, false);
                        break;
                }
        } break;
        case ColliderType_CIRCLE: {
                switch (B.colliderType) {
                case ColliderType_POINT:
                        return checkCollision_PointCircle(
                            data,
                            *(PointColliderData *)B.colliderData,
                            B_T, B_velocity,
                            *(CircleColliderData *)A.colliderData,
                            A_T, A_velocity, true);
                        break;
                case ColliderType_BOX:
                        return checkCollision_BoxCircle(
                            data,
                            *(BoxColliderData *)B.colliderData,
                            B_T, B_velocity,
                            *(CircleColliderData *)A.colliderData,
                            A_T, A_velocity, true);
                        break;
                case ColliderType_CIRCLE:
                        return checkCollision_CircleCircle(
                            data,
                            *(CircleColliderData *)A.colliderData,
                            A_T, A_velocity,
                            *(CircleColliderData *)B.colliderData,
                            B_T, B_velocity, false);
                        break;
                case ColliderType_GROUP:
                        return checkCollision_CircleGroup(
                            data,
                            *(CircleColliderData *)A.colliderData,
                            A_T, A_velocity,
                            *(GroupColliderData *)B.colliderData,
                            B_T, B_velocity, false);
                        break;
                }
        } break;
        case ColliderType_GROUP: {
                switch (B.colliderType) {
                case ColliderType_POINT:
                        return checkCollision_PointGroup(
                            data,
                            *(PointColliderData *)B.colliderData,
                            B_T, B_velocity,
                            *(GroupColliderData *)A.colliderData,
                            A_T, A_velocity, true);
                        break;
                case ColliderType_BOX:
                        return checkCollision_BoxGroup(
                            data,
                            *(BoxColliderData *)B.colliderData,
                            B_T, B_velocity,
                            *(GroupColliderData *)A.colliderData,
                            A_T, A_velocity, true);
                        break;
                case ColliderType_CIRCLE:
                        return checkCollision_CircleGroup(
                            data,
                            *(CircleColliderData *)B.colliderData,
                            B_T, B_velocity,
                            *(GroupColliderData *)A.colliderData,
                            A_T, A_velocity, true);
                        break;
                case ColliderType_GROUP:
                        return checkCollision_GroupGroup(
                            data,
                            *(GroupColliderData *)A.colliderData,
                            A_T, A_velocity,
                            *(GroupColliderData *)B.colliderData,
                            B_T, B_velocity, false);
                        break;
                }
        } break;
        }
        return false;
}

bool checkCollision_PointPoint(CollisionData *data, PointColliderData A, Transform2D A_T, Vector2 A_velocity, PointColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal) {
        // If we reach here that means they've passed the bounding boxes, we know for sure they've hit.
        // Just calc the normal and we're done.
        // How in the world did two point masses collide?
        data->travel = 0.f;
        data->point = applyTransform(A_T, A.offset);
        Vector2 surface = Vector2Add(Vector2Normalize(A_velocity), Vector2Normalize(B_velocity));
        if (Vector2Equals(surface, Vector2Zero())) {
                data->normal = B_velocity;
        } else {
                surface = Vector2Normalize(surface);
                data->normal = invertNormal ? (Vector2){surface.y, -surface.x} : (Vector2){-surface.y, surface.x};
        }
        return true;
}

bool checkCollision_PointBox(CollisionData *data, PointColliderData A, Transform2D A_T, Vector2 A_velocity, BoxColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal) {
        // Send out a ray and do the whole "Odd or even number of intersections" thing
        // The ray's origin is the point, the direction is the inverse of it's relative velocity
        // That relative velocity would just be A_vel - B_vel, and so the inverse is B_v - A_v

        Vector2 A_pos = applyTransform(A_T, A.offset);
        Vector2 inv_relative_vel = Vector2Subtract(B_velocity, A_velocity);

        RectPoints vertices = calculateRectanglePoints(B, B_T);

        RayCastLineHit e0 = rayCast_line(A_pos, inv_relative_vel, vertices.a, vertices.b);
        RayCastLineHit e1 = rayCast_line(A_pos, inv_relative_vel, vertices.b, vertices.c);
        RayCastLineHit e2 = rayCast_line(A_pos, inv_relative_vel, vertices.c, vertices.d);
        RayCastLineHit e3 = rayCast_line(A_pos, inv_relative_vel, vertices.d, vertices.a);

        // XOR to get parity of hits
        bool collision = e0.hit ^ e1.hit ^ e2.hit ^ e3.hit;
        if (!collision) {
                return false;
        }

        // Collided, so now find the collision point
        // As it's a rectangle, should only have one collision
        int hit = (e0.hit ? 0 : (e1.hit ? 1 : (e2.hit ? 2 : 3)));
        // Get the vertices of the edge again, and the t
        Vector2 a, b;
        float t;
        switch (hit) {
        case 0:
                a = vertices.a;
                b = vertices.b;
                t = e0.t;
                break;
        case 1:
                a = vertices.b;
                b = vertices.c;
                t = e1.t;
                break;
        case 2:
                a = vertices.c;
                b = vertices.d;
                t = e2.t;
                break;
        case 3:
                a = vertices.d;
                b = vertices.a;
                t = e3.t;
                break;
        }
        // We know that they're wound CCW because I was goated when writing the function
        Vector2 edgeDiff = Vector2Subtract(a, b);
        float invEdgeDiffLen = 1.f / Vector2Length(edgeDiff);
        data->normal = rotateQuarterTurn(Vector2Scale(edgeDiff, invEdgeDiffLen));
        if (invertNormal)
                data->normal = Vector2Invert(data->normal);
        data->point = Vector2Add(A_pos, Vector2Scale(inv_relative_vel, t));
        // For the travel in this case, we have to see how much to move the point by to get it out of the rectangle
        // from the side it came in, I.E. the distance between the point and the line
        data->travel = abs((b.y - a.y) * A_pos.x - (b.x - a.x) * A_pos.y + b.x * a.y - b.y * a.x) / invEdgeDiffLen;

        return true;
}

bool checkCollision_PointCircle(CollisionData *data, PointColliderData A, Transform2D A_T, Vector2 A_velocity, CircleColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal) {
        // The checking for radius and stuff is nice for finding out *if* there was a hit,
        // but not for the collision point and the normal. All three come out nicely with
        // just a raycast.

        Vector2 A_pos = applyTransform(A_T, A.offset);
        Vector2 B_pos = applyTransform(B_T, B.offset);
        Vector2 inv_relative_vel = Vector2Subtract(B_velocity, A_velocity);
        Vector2 relative_pos = Vector2Subtract(A_pos, B_pos);
        RayCastCircleHit cast = rayCast_circle(relative_pos, inv_relative_vel, B.radius);

        if (!cast.hit || cast.t0 >= 0)
                return false;

        Vector2 rel_point = Vector2Add(relative_pos, Vector2Scale(inv_relative_vel, cast.t1));
        data->point = Vector2Add(rel_point, B_pos);
        data->normal = Vector2Normalize(rel_point);
        if (invertNormal)
                data->normal = Vector2Invert(data->normal);
        data->travel = B.radius - Vector2Length(relative_pos);

        return true;
}

bool checkCollision_PointGroup(CollisionData *data, PointColliderData A, Transform2D A_T, Vector2 A_velocity, GroupColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal) {
        // Now we individually check for collisions between each group member
        // Because this is a point, it should only collide with one group member
        // And if it doesn't, it's a programmer issue
        // So we can just return on the first hit, since we know it's the "right" one
        for (int i = 0; i < B.n; i++) {
                switch (B.types[i]) {
                case ColliderType_POINT:
                        if (checkCollision_PointPoint(data,
                                                      A, A_T, A_velocity,
                                                      *(PointColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                      invertNormal))
                                return true;
                        break;
                case ColliderType_BOX:
                        if (checkCollision_PointBox(data,
                                                    A, A_T, A_velocity,
                                                    *(BoxColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                    invertNormal))
                                return true;
                        break;
                case ColliderType_CIRCLE:
                        if (checkCollision_PointCircle(data,
                                                       A, A_T, A_velocity,
                                                       *(CircleColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                       invertNormal))
                                return true;
                        break;
                case ColliderType_GROUP:
                        if (checkCollision_PointGroup(data,
                                                      A, A_T, A_velocity,
                                                      *(GroupColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                      invertNormal))
                                return true;
                        break;
                }
        }

        return false;
}

/* Abandoned Separating Axis Theorem Stuff
// typedef struct SeperatingAxisData {
//         bool isSeperating;
//         int aminp;
//         int amaxp;
//         int bminp;
//         int bmaxp;
// } SeperatingAxisData;
// SeperatingAxisData isSeparatingAxis(RectPoints A_points, RectPoints B_points, Vector2 axis) {
//
//         // First, project all the points onto the axis
//         // A 3b1b video I watched a while ago shows that
//         // Projecting v onto w's axis is just v*w / ||w||,
//         // And ||w|| is a constant so we can ignore it for this
//         // purpose; Just use v*w
//
//         float project;
//         // Point A
//         int p = 0;
//         int aminp = p;
//         float amin = Vector2DotProduct(A_points.a, axis);
//         int amaxp = p;
//         float amax = amin;
//         int bminp = p;
//         float bmin = Vector2DotProduct(B_points.a, axis);
//         int bmaxp = p;
//         float bmax = bmin;
//         // Point B
//         int p = 1;
//         project = Vector2DotProduct(A_points.b, axis);
//         aminp = (amin < project) ? aminp : p;
//         amin = (amin < project) ? amin : project;
//         amaxp = (amax > project) ? amaxp : p;
//         amax = (amax > project) ? amax : project;
//         project = Vector2DotProduct(B_points.b, axis);
//         bminp = (bmin < project) ? bminp : p;
//         bmin = (bmin < project) ? bmin : project;
//         bmaxp = (bmax > project) ? bmaxp : p;
//         bmax = (bmax > project) ? bmax : project;
//         // Point C
//         int p = 2;
//         project = Vector2DotProduct(A_points.c, axis);
//         aminp = (amin < project) ? aminp : p;
//         amin = (amin < project) ? amin : project;
//         amaxp = (amax > project) ? amaxp : p;
//         amax = (amax > project) ? amax : project;
//         project = Vector2DotProduct(B_points.c, axis);
//         bminp = (bmin < project) ? bminp : p;
//         bmin = (bmin < project) ? bmin : project;
//         bmaxp = (bmax > project) ? bmaxp : p;
//         bmax = (bmax > project) ? bmax : project;
//         // Point D
//         int p = 3;
//         project = Vector2DotProduct(A_points.d, axis);
//         aminp = (amin < project) ? aminp : p;
//         amin = (amin < project) ? amin : project;
//         amaxp = (amax > project) ? amaxp : p;
//         amax = (amax > project) ? amax : project;
//         project = Vector2DotProduct(B_points.d, axis);
//         bminp = (bmin < project) ? bminp : p;
//         bmin = (bmin < project) ? bmin : project;
//         bmaxp = (bmax > project) ? bmaxp : p;
//         bmax = (bmax > project) ? bmax : project;

//         // Now just an intersection check
//         return (SeperatingAxisData){amax < bmin || amin > bmax, aminp, amaxp, bminp, bmaxp};
// }
*/

bool checkCollision_BoxBox(CollisionData *data, BoxColliderData A, Transform2D A_T, Vector2 A_velocity, BoxColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal) {
        RectPoints A_pts = calculateRectanglePoints(A, A_T);
        RectPoints B_pts = calculateRectanglePoints(B, B_T);

        // I could do fancy stuff with the seperating axis theorem, but I can't
        // figure out then how to find the exact collision point and normal and stuff like that.
        // So, just check if each point is inside the other rectangle
        // Calculate the travel (kinda) for each point, and the point with the most travel is taken as
        // the main "colliding point" for the CollisionData
        // Yeah yeah, this doesn't test all intersections, but in a practical usage, yes it does.

        Vector2 invAVel = Vector2Invert(A_velocity);
        Vector2 invBVel = Vector2Invert(B_velocity);

        Vector2 *avs = &A_pts;
        Vector2 *bvs = &B_pts;

        int winningAorB = -1;
        int winningPt;
        int winningEdgeA;
        int winningEdgeB;
        float winningT;

        for (int i = 0; i < 4; i++) {
                // Check if point a[i] is in b
                bool sum = false;
                int newj;
                float newT;
                for (int j = 0; j < 3; j++) {
                        int j1 = (j + 1) % 3;
                        RayCastLineHit rc = rayCast_line(avs[i], invAVel, bvs[j], bvs[j1]);
                        if (!rc.hit)
                                continue;
                        if (sum)
                                goto checkB;
                        sum = true;
                        newj = j;
                        newT = rc.t;
                }
                if ((sum % 2) && (winningAorB == -1 || winningT > newT)) {
                        winningAorB = 0;
                        winningPt = i;
                        winningEdgeA = newj;
                        winningEdgeB = (newj + 1) % 3;
                        winningT = newT;
                }
        checkB:
                sum = false;
                for (int j = 0; j < 3; j++) {
                        int j1 = (j + 1) % 3;
                        RayCastLineHit rc = rayCast_line(bvs[i], invBVel, avs[j], avs[j1]);
                        if (!rc.hit)
                                continue;
                        if (sum)
                                goto nextCycle;
                        sum = true;
                        newj = j;
                        newT = rc.t;
                }
                if ((sum % 2) && (winningAorB == -1 || winningT > newT)) {
                        winningAorB = 1;
                        winningPt = i;
                        winningEdgeA = newj;
                        winningEdgeB = (newj + 1) % 3;
                        winningT = newT;
                }
        nextCycle:
        }
        if (winningAorB == -1)
                return false;

        // Reusing code from point-line
        Vector2 *vs = winningAorB ? bvs : avs;
        Vector2 a = vs[winningEdgeA];
        Vector2 b = vs[winningEdgeB];
        Vector2 landed = vs[winningPt];
        Vector2 edgeDiff = Vector2Subtract(a, b);
        float invEdgeDiffLen = 1.f / Vector2Length(edgeDiff);
        data->normal = rotateQuarterTurn(Vector2Scale(edgeDiff, invEdgeDiffLen));
        if (invertNormal ^ winningAorB)
                data->normal = Vector2Invert(data->normal);
        data->point = Vector2Add(landed, Vector2Scale(winningAorB ? invBVel : invAVel, winningT));
        // For the travel in this case, we have to see how much to move the point by to get it out of the rectangle
        // from the side it came in, I.E. the distance between the point and the line
        data->travel = abs((b.y - a.y) * landed.x - (b.x - a.x) * landed.y + b.x * a.y - b.y * a.x) / invEdgeDiffLen;

        return true;
}

float calculateCornerDistanceSqr(Vector2 p, float t, Vector2 a, Vector2 b) {
        return Vector2DistanceSqr(p, (t < 0.5f) ? a : b);
}

bool checkCollision_BoxCircle(CollisionData *data, BoxColliderData A, Transform2D A_T, Vector2 A_velocity, CircleColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal) {
        // This thing again

        Vector2 B_pos = applyTransform(B_T, B.offset);

        RectPoints A_pts = calculateRectanglePoints(A, A_T);
        int hitEdge = 0; // Not really hit, moreso "Is closer to an edge than it's corners, don't bother checking corners later"
        Vector2 hitPoint;
        float d;
        // Calculate t and distance for each side, getting the minimum
        // ngl this formula is elegant AF
        Vector2 edge = Vector2Subtract(A_pts.b, A_pts.a);
        float t_A = -Vector2DotProduct(Vector2Subtract(B_pos, A_pts.a), edge) / Vector2LengthSqr(edge);
        if (0.f <= t_A && t_A <= 1.f) {
                Vector2 pt = Vector2Add(A_pts.a, Vector2Scale(edge, t_A));
                float distSqr = Vector2DistanceSqr(B_pos, pt);
                hitEdge = 1;
                hitPoint = pt;
                d = distSqr;
        }
        edge = Vector2Subtract(A_pts.c, A_pts.b);
        float t_B = -Vector2DotProduct(Vector2Subtract(B_pos, A_pts.b), edge) / Vector2LengthSqr(edge);
        if (0.f <= t_B && t_B <= 1.f) {
                Vector2 pt = Vector2Add(A_pts.b, Vector2Scale(edge, t_B));
                float distSqr = Vector2DistanceSqr(B_pos, pt);
                if (d < distSqr) {
                        d = distSqr;
                        hitPoint = pt;
                        hitEdge = 2;
                }
        }
        edge = Vector2Subtract(A_pts.d, A_pts.c);
        float t_C = -Vector2DotProduct(Vector2Subtract(B_pos, A_pts.c), edge) / Vector2LengthSqr(edge);
        if (0.f <= t_B && t_B <= 1.f) {
                Vector2 pt = Vector2Add(A_pts.c, Vector2Scale(edge, t_C));
                float distSqr = Vector2DistanceSqr(B_pos, pt);
                if (d < distSqr) {
                        d = distSqr;
                        hitPoint = pt;
                        hitEdge = 3;
                }
        }
        edge = Vector2Subtract(A_pts.a, A_pts.d);
        float t_D = -Vector2DotProduct(Vector2Subtract(B_pos, A_pts.d), edge) / Vector2LengthSqr(edge);
        if (0.f <= t_B && t_B <= 1.f) {
                Vector2 pt = Vector2Add(A_pts.d, Vector2Scale(edge, t_D));
                float distSqr = Vector2DistanceSqr(B_pos, pt);
                if (d < distSqr) {
                        d = distSqr;
                        hitPoint = pt;
                        hitEdge = 4;
                }
        }

        if (hitEdge) {
                // If it hit any edge, it's for sure closer to one of the edges it hit than any of the corners.
                // So, we can skip over comparing their distances and stuff.
                float travelSqr = B.radius * B.radius - d;
                if (travelSqr < 0.f)
                        return false;

                data->point = hitPoint;
                data->travel = sqrtf(travelSqr);
                switch (hitEdge) {
                case 1:
                        edge = Vector2Subtract(A_pts.b, A_pts.a);
                        break;
                case 2:
                        edge = Vector2Subtract(A_pts.c, A_pts.b);
                        break;
                case 3:
                        edge = Vector2Subtract(A_pts.d, A_pts.c);
                        break;
                case 4:
                        edge = Vector2Subtract(A_pts.a, A_pts.d);
                        break;
                }

                data->normal = Vector2Scale(rotateQuarterTurn(Vector2Normalize(edge)), invertNormal ? 1 : -1);

                return true;
        }

        // Now check the corners
        int hitCorner = 0;
        float d_A = calculateCornerDistanceSqr(B_pos, t_A, A_pts.a, A_pts.b);
        d = d_A;
        hitCorner = 1;
        float d_B = calculateCornerDistanceSqr(B_pos, t_B, A_pts.b, A_pts.c);
        if (d_B < d) {
                d = d_B;
                hitCorner = 2;
        }
        float d_C = calculateCornerDistanceSqr(B_pos, t_C, A_pts.c, A_pts.d);
        if (d_C < d) {
                d = d_C;
                hitCorner = 3;
        }
        float d_D = calculateCornerDistanceSqr(B_pos, t_D, A_pts.d, A_pts.a);
        if (d_D < d) {
                d = d_D;
                hitCorner = 4;
        }

        float travelSqr = B.radius * B.radius - d;
        if (travelSqr < 0.f)
                return false;

        Vector2 corner;
        switch (hitCorner) {
        case 1:
                corner = A_pts.a;
                break;
        case 2:
                corner = A_pts.b;
                break;
        case 3:
                corner = A_pts.c;
                break;
        case 4:
                corner = A_pts.d;
                break;
        }

        data->travel = sqrtf(travelSqr);
        data->point = corner;
        data->normal = Vector2Scale(Vector2Subtract(corner, B_pos), invertNormal ? 1 : -1);

        return true;
}

bool checkCollision_BoxGroup(CollisionData *data, BoxColliderData A, Transform2D A_T, Vector2 A_velocity, GroupColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal) {
        // Find just the first collision
        // Again, practicality: If they're any real problems, they'll likely be fixed in following frames

        for (int i = 0; i < B.n; i++) {
                for (int i = 0; i < B.n; i++) {
                        switch (B.types[i]) {
                        case ColliderType_POINT:
                                if (checkCollision_PointBox(data,
                                                            *(PointColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                            A, A_T, A_velocity,
                                                            !invertNormal))
                                        return true;
                                break;
                        case ColliderType_BOX:
                                if (checkCollision_BoxBox(data,
                                                          A, A_T, A_velocity,
                                                          *(BoxColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                          invertNormal))
                                        return true;
                                break;
                        case ColliderType_CIRCLE:
                                if (checkCollision_BoxCircle(data,
                                                             A, A_T, A_velocity,
                                                             *(CircleColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                             invertNormal))
                                        return true;
                                break;
                        case ColliderType_GROUP:
                                if (checkCollision_BoxGroup(data,
                                                            A, A_T, A_velocity,
                                                            *(GroupColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                            invertNormal))
                                        return true;
                                break;
                        }
                }

                return false;
        }
}

bool checkCollision_CircleCircle(CollisionData *data, CircleColliderData A, Transform2D A_T, Vector2 A_velocity, CircleColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal) {
        // Probably the easiest case of the bunch
        Vector2 A_pos = applyTransform(A_T, A.offset);
        Vector2 B_pos = applyTransform(B_T, B.offset);

        float d2 = Vector2DistanceSqr(A_pos, B_pos);
        float sr = A.radius + B.radius;
        float travelSqr = sr * sr - d2;
        if (travelSqr < 0.f)
                return false;

        data->travel = sqrtf(travelSqr);
        data->point = Vector2Scale(Vector2Add(A_pos, B_pos), 0.5);
        data->normal = Vector2Scale(Vector2Subtract(A_pos, B_pos), invertNormal ? 1 : -1);
        return true;
}

bool checkCollision_CircleGroup(CollisionData *data, CircleColliderData A, Transform2D A_T, Vector2 A_velocity, GroupColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal) {
        // Same thing as the box collider. Physics, tiny steps, e.t.c.

        for (int i = 0; i < B.n; i++) {
                for (int i = 0; i < B.n; i++) {
                        switch (B.types[i]) {
                        case ColliderType_POINT:
                                if (checkCollision_PointCircle(data,
                                                               *(PointColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                               A, A_T, A_velocity,
                                                               !invertNormal))
                                        return true;
                                break;
                        case ColliderType_BOX:
                                if (checkCollision_BoxCircle(data,
                                                             *(BoxColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                             A, A_T, A_velocity,
                                                             !invertNormal))
                                        return true;
                                break;
                        case ColliderType_CIRCLE:
                                if (checkCollision_CircleCircle(data,
                                                                A, A_T, A_velocity,
                                                                *(CircleColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                                invertNormal))
                                        return true;
                                break;
                        case ColliderType_GROUP:
                                if (checkCollision_CircleGroup(data,
                                                               A, A_T, A_velocity,
                                                               *(GroupColliderData *)B.colliderDatas[i], B_T, B_velocity,
                                                               invertNormal))
                                        return true;
                                break;
                        }
                }

                return false;
        }
}

bool checkCollision_GroupGroup(CollisionData *data, GroupColliderData A, Transform2D A_T, Vector2 A_velocity, GroupColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal) {
        // Every day, I regret more and more adding group colliders

        for (int i = 0; i < A.n; i++) {
                for (int i = 0; i < A.n; i++) {
                        switch (A.types[i]) {
                        case ColliderType_POINT:
                                if (checkCollision_PointGroup(data,
                                                              *(PointColliderData *)A.colliderDatas[i], A_T, A_velocity,
                                                              B, B_T, B_velocity,
                                                              invertNormal))
                                        return true;
                                break;
                        case ColliderType_BOX:
                                if (checkCollision_BoxGroup(data,
                                                            *(BoxColliderData *)A.colliderDatas[i], A_T, A_velocity,
                                                            B, B_T, B_velocity,
                                                            invertNormal))
                                        return true;
                                break;
                        case ColliderType_CIRCLE:
                                if (checkCollision_CircleGroup(data,
                                                               *(CircleColliderData *)A.colliderDatas[i], A_T, A_velocity,
                                                               B, B_T, B_velocity,
                                                               invertNormal))
                                        return true;
                                break;
                        case ColliderType_GROUP:
                                if (checkCollision_GroupGroup(data,
                                                              *(GroupColliderData *)A.colliderDatas[i], A_T, A_velocity,
                                                              B, B_T, B_velocity,
                                                              invertNormal))
                                        return true;
                                break;
                        }
                }

                return false;
        }
}

RayCastLineHit rayCast_line(Vector2 rayOrigin, Vector2 rayDirection, Vector2 lineA, Vector2 lineB) {

        Vector2 diff = Vector2Subtract(lineB, lineA);

        float t =
            (diff.x * (rayOrigin.y - lineA.y) - (rayOrigin.x - rayOrigin.y) * diff.y) /
            (diff.y * rayDirection.x - diff.x * rayDirection.y);

        float u = (rayDirection.x * t + rayOrigin.x - lineA.x) / diff.x;

        return (RayCastLineHit){
            (t >= 0.f) && (u >= 0.f) && (u <= 1.f),
            t,
            u,
        };
}

RayCastCircleHit rayCast_circle(Vector2 rayOrigin, Vector2 rayDirection, float radius) {
        // Turns into a quadratic and stuff, if discriminant < 0 then it doesn't hit
        // Two solutions for t; If one is negative, then the origin is inside the circle

        float dot = Vector2DotProduct(rayOrigin, rayDirection);
        float Dsqr = Vector2LengthSqr(rayDirection);
        float discriminant = dot * dot - Dsqr * (Vector2LengthSqr(rayOrigin) - radius * radius);

        if (discriminant < 0) {
                return (RayCastCircleHit){false, 0, 0};
        }

        float discSqrt = sqrtf(discriminant);

        return (RayCastCircleHit){
            true,
            (-dot + discSqrt) / Dsqr,
            (-dot - discSqrt) / Dsqr,
        };
}
