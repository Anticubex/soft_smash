#include "physics.h"

typedef struct SoftBodyRenderer {
        // A surface following how to render the softbody
        int num;
        int *pts;
        Color fillColor;
        Color borderColor;
        float thickness;
} SoftBodyRenderer;
void freeRenderer(SoftBodyRenderer *rend);

void renderSoftbody(SoftBody sb, SoftBodyRenderer rend);

void autogenerateRendererFromSurface(SoftBody sb, SoftBodyRenderer *rend);
