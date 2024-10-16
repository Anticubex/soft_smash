#ifndef COLLISION_H_
#define COLLISION_H_

#include "physics.h"

typedef enum SoftBodyMaterial {
        SoftBodyMaterial_DEFAULT
} SoftBodyMaterial;

/* Collision */
// TODO: Make this multipoint, single-point doesn't cut it
typedef struct CollisionData {
        bool collided;
        // The struct assumes a point from A is inside B;
        // This inverts that assumption;
        bool invert;
        int point;
        int edge;
        float edge_t;
        float distance;
        Vector2 nearest;
} CollisionData;

// Note: Collision checking only works if the bodies
// are made very properly--correctly wound and closed
// surfaces, e.t.c.
// i.e. if the softbody doesn't have all its surfaces
// closed and wound CCW it doesn't work
CollisionData checkCollision(SoftBody A, SoftBody B);

float getFriction(SoftBodyMaterial A, SoftBodyMaterial B);

// char *debugString = ((void *)0);

// Handles collisions, applying forces to each, e.t.c.
void handleCollision(SoftBody A, SoftBody B, CollisionData data, SoftBodyMaterial matA, SoftBodyMaterial matB, float dt);

#endif // COLLISION_H_