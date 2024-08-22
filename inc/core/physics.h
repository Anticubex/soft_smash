#ifndef PHYSICS_H
#define PHYSICS_H

#include <raylib.h>
#include <raymath.h>

typedef struct WorldValues {
        Vector2 gravity;
        float airPressure;
} WorldValues;

typedef struct BB {
        Vector2 min;
        Vector2 max;
} BB;

// Flags, can be both
typedef enum SoftBodyType {
        SoftBodyType_Shape = 0b001,
        SoftBodyType_Pressure = 0b010,
} SoftBodyType;
typedef struct SoftBody {
        SoftBodyType type;
        int numPoints;
        Vector2 *pointPos;
        Vector2 *pointVel;
        Vector2 *shape; // Make sure the frame is balanced (the average of the points is the origin), or else (i think) the body will move on its own
        Vector2 shapePosition;
        float shapeRotation;
        // Surfaces wind CCW mathematically, but because Y is inverted,
        // this means that on-screen they wind CW
        int numSurfaces;
        int *surfaceA;
        int *surfaceB;
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

void update_SoftBody(SoftBody *sb, WorldValues worldValues, float dt);
void SBPoint_addForce(SoftBody *sb, int i, Vector2 force, float dt);

SoftBody createEmptySoftBody(
    SoftBodyType type,
    float mass,
    float linearDrag,
    float springStrength,
    float springDamp,
    float shapeSpringStrength,
    float nRT);
void freeSoftbody(SoftBody *toFree);

// private util functions
void _center_sb_shape(SoftBody *sb);
void _alloc_sb(SoftBody *sb, int numPoints, int numSurfaces, int numSprings);

void circleSoftbody(SoftBody *sb, Vector2 center, float radius, int numPoints);
void rectSoftbody(SoftBody *sb, Vector2 center, Vector2 scale, int detailX, int detailY, bool makeTruss);

#endif