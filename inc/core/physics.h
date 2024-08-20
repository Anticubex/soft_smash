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
        BB bounds;
} SoftBody;

void update_PointBody(PointBody pb, WorldValues worldValues, float dt);
void update_SoftBody(SoftBody sb, WorldValues worldValues, float dt);

void PointBody_applyForce(PointBody *pb, Vector2 force, float dt);

typedef struct BB {
        Vector2 min;
        Vector2 max;
} BB;

#endif