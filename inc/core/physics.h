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
        SoftBodyType_Springs = 0b100,
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

// helpers for RK4
// For the calcForce functions, they add onto a list of vectors with the
// Forces on each point from each force. They use the SoftBody sb only
// for it's parameters; they use the SBPoints points as the actual position
// of the softbody
typedef struct SBPoints {
        int num;
        Vector2 *pos;
        Vector2 *vel;
} SBPoints;
void calcForces(Vector2 *forces, SoftBody sb, SBPoints points, WorldValues worldValues);
void calcForce_springs(Vector2 *forces, SoftBody sb, SBPoints points, WorldValues worldValues);
void calcForce_shape(Vector2 *forces, SoftBody sb, SBPoints points, WorldValues worldValues);
void calcForce_pressure(Vector2 *forces, SoftBody sb, SBPoints points, WorldValues worldValues);
void calcForce_drag(Vector2 *forces, SoftBody sb, SBPoints points, WorldValues worldValues);
void projectSB(SBPoints *dest, SBPoints src, Vector2 *forces, float dt);

typedef struct SBPos {
        Vector2 position;
        float rotation;
} SBPos;
// Calculates and stores the position and rotation of the softbody (as for shape matching)
SBPos calcShape(SoftBody sb, SBPoints points);

Vector2 *alloc_forces(int num);
void sumForces(int num, Vector2 *forces, Vector2 *other, float multiplier);

SBPoints rip_SBPoints(SoftBody sb);
void apply_SBPoints(SoftBody *sb, SBPoints points);
void alloc_SBPoints(SBPoints *points, int num);
void free_SBPoints(SBPoints *points);

void circleSoftbody(SoftBody *sb, Vector2 center, float radius, int numPoints);
void rectSoftbody(SoftBody *sb, Vector2 center, Vector2 scale, int detailX, int detailY, bool makeTruss);

#endif