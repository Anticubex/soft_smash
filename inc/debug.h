#ifndef DEBUG_H_
#define DEBUG_H_

#include <core/physics.h>

Color interpolate3way(Color A, Color B, Color C, float t);
void DrawSoftbody_debug(SoftBody sb);

#endif // DEBUG_H_