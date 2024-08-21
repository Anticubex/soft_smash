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

// Flags, can be both
typedef enum SoftBodyType {
        SoftBodyType_Shape = 0b001,
        SoftBodyType_Pressure = 0b010,
} SoftBodyType;
typedef struct SoftBody {
        SoftBodyType type;
        int numPoints;
        // Points wind CCW mathematically, but because Y is inverted,
        // this means that on-screen they wind CW
        Vector2 *pointPos;
        Vector2 *pointVel;
        Vector2 *shape; // Make sure the frame is balanced (the average of the points is the origin), or else (i think) the body will move on its own
        int numSprings;
        int *springA;
        int *springB;
        float *lengths;
        float mass;
        float linearDrag;
        float springStrength;
        float springDamp;
        float shapeSpringStrength;
        float nRT;
        BB bounds;
} SoftBody;

void update_SoftBody(SoftBody sb, WorldValues worldValues, float dt);

typedef struct BB {
        Vector2 min;
        Vector2 max;
} BB;

SoftBody createEmptySoftBody(
    SoftBodyType type,
    float mass,
    float linearDrag,
    float springStrength,
    float springDamp,
    float shapeSpringStrength,
    float nRT);
void freeSoftbody(SoftBody *toFree);

// private util function
void _alloc_sb(SoftBody *sb, int numPoints, int numSprings);

void circleSoftbody(SoftBody *sb, Vector2 center, float radius, int numPoints);
void rectSoftbody(SoftBody *sb, Vector2 center, Vector2 scale, int detailX, int detailY, bool makeTruss);

#endif