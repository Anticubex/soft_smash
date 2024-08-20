#ifndef PHYSICS_H
#define PHYSICS_H

#include "core.h"
#include <raylib.h>
#include <raymath.h>

typedef struct WorldValues {
        Vector2 gravity;
        float airDensity;
        float airPressure;
        float linearDrag;
        float angularDrag;
} WorldValues;

typedef struct PhysicsMaterial {
        float epsilon;
} PhysicsMaterial;

typedef struct RigidBody {
        Transform2D transform;
        Vector2 velocity;
        float angularVelocity;
        float mass;
        float invMass; // Inverse of mass
        float momentOfInertia;
        float invMomentOfInertia;
        // Negative values indicate no override of World Values
        float linearDrag;
        float angularDrag;
} RigidBody;

typedef struct PointBody {
        Vector2 position;
        Vector2 velocity;
        float mass;
        float invMass; // Inverse of mass
        // Negative values indicate no override of World Values
        float linearDrag;
} PointBody;

// Flags, can be both
typedef enum SoftBodyType {
        SoftBodyType_Shape = 0b001,
        SoftBodyType_Pressure = 0b010,
        SoftBodyType_SelfColliding = 0b100,
} SoftBodyType;
typedef struct SoftBody {
        SoftBodyType type;
        int num_points;
        PointBody *points; // Points wind CCW, only matters for pressure
        Vector2 *shape;    // Make sure the frame is balanced (the average of the points is the origin), or else (i think) the body will move on its own
        int num_springs;
        int *springA;
        int *springB;
        float *lengths;
        float linearDrag;
        float springStrength;
        float springDamp;
        float shapeSpringStrength;
        float nRT;
        float radii;
        BB bounds;
} SoftBody;

void update_RigidBody(RigidBody rb, WorldValues worldValues, float dt);
void update_PointBody(PointBody pb, WorldValues worldValues, float dt);
void update_SoftBody(SoftBody sb, WorldValues worldValues, float dt);

void PointBody_applyForce(PointBody *pb, Vector2 force, float dt);

typedef struct BB {
        Vector2 min;
        Vector2 max;
} BB;

// Note: SoftBodies have their own bespoke collision, but that's a TODO
typedef enum ColliderType {
        ColliderType_POINT,
        ColliderType_BOX,
        ColliderType_CIRCLE,
        ColliderType_GROUP,
} ColliderType;
typedef struct Collider {
        // Bounding box
        BB bounds;
        ColliderType colliderType;
        void *colliderData;
} Collider;

typedef struct PointColliderData {
        Vector2 offset;
} PointColliderData; // Literaly is just the min of the bounding box
typedef struct BoxColliderData {
        Vector2 center;
        Vector2 dimensions;
        float rotation;
        // TODO: Every single function that uses this data just calculates the points, let's embed them
} BoxColliderData;
typedef struct CircleColliderData {
        Vector2 offset;
        float radius;
} CircleColliderData;
typedef struct GroupColliderData {
        int n;
        ColliderType *types;
        void **colliderDatas;
} GroupColliderData;

// Normal goes A <- B
typedef struct CollisionData {
        // TODO: Find more stuff to put in here
        float travel;
        Vector2 point;
        Vector2 normal;
} CollisionData;

typedef struct __attribute__((packed)) RectPoints {
        Vector2 a;
        Vector2 b;
        Vector2 c;
        Vector2 d;
} RectPoints;
RectPoints calculateRectanglePoints(BoxColliderData colliderData, Transform2D transform);

Collider initPointCollider(Vector2 offset);
Collider initBoxCollider(Vector2 center, Vector2 dimensions, float rotation);
Collider initCircleCollider(Vector2 offset, float radius);
Collider initGroupCollider(int n);

void freeCollider(Collider collider);

BB getPointBB(PointColliderData data, Transform2D transform);
BB getRectBB(BoxColliderData data, Transform2D transform);
BB getCircleBB(CircleColliderData data, Transform2D transform);
BB getGroupBB(GroupColliderData data, Transform2D transform);

BB findColliderBB(Collider collider, Transform2D transform);

bool checkCollision(CollisionData *data, Collider A, Transform2D A_T, Vector2 A_velocity, Collider B, Transform2D B_T, Vector2 B_velocity);

// `invertNormal` is to account for how the function flips it internally sometimes

bool checkCollision_PointPoint(CollisionData *data, PointColliderData A, Transform2D A_T, Vector2 A_velocity, PointColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal);
bool checkCollision_PointBox(CollisionData *data, PointColliderData A, Transform2D A_T, Vector2 A_velocity, BoxColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal);
bool checkCollision_PointCircle(CollisionData *data, PointColliderData A, Transform2D A_T, Vector2 A_velocity, CircleColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal);
bool checkCollision_PointGroup(CollisionData *data, PointColliderData A, Transform2D A_T, Vector2 A_velocity, GroupColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal);

bool checkCollision_BoxBox(CollisionData *data, BoxColliderData A, Transform2D A_T, Vector2 A_velocity, BoxColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal);
bool checkCollision_BoxCircle(CollisionData *data, BoxColliderData A, Transform2D A_T, Vector2 A_velocity, CircleColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal);
bool checkCollision_BoxGroup(CollisionData *data, BoxColliderData A, Transform2D A_T, Vector2 A_velocity, GroupColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal);

bool checkCollision_CircleCircle(CollisionData *data, CircleColliderData A, Transform2D A_T, Vector2 A_velocity, CircleColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal);
bool checkCollision_CircleGroup(CollisionData *data, CircleColliderData A, Transform2D A_T, Vector2 A_velocity, GroupColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal);

bool checkCollision_GroupGroup(CollisionData *data, GroupColliderData A, Transform2D A_T, Vector2 A_velocity, GroupColliderData B, Transform2D B_T, Vector2 B_velocity, bool invertNormal);

// Let the ray and the segment be represented as two
// parametric figures.
// The ray is `rayOrigin + rayDirection * t`
// The line is `lineA + (lineB - lineA) * u`
typedef struct RayCastLineHit {
        bool hit;
        float t;
        float u;
} RayCastLineHit;

RayCastLineHit rayCast_line(Vector2 rayOrigin, Vector2 rayDirection, Vector2 lineA, Vector2 lineB);

typedef struct RayCastCircleHit {
        bool hit;
        float t0;
        float t1;
} RayCastCircleHit;
RayCastCircleHit rayCast_circle(Vector2 rayOrigin, Vector2 rayDirection, float radius);

#endif