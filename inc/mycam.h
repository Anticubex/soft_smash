#ifndef MYCAM_H
#define MYCAM_H

#include <raylib.h>
#include <raymath.h>

typedef struct {
        Vector2 center;
        // The size in world space (vertically only, to preserve aspect ratio)
        float v_size;
        int screenWidth;
        int screenHeight;
        float rotation;
        Camera2D raylib_cam;
        // Now for the more private, efficiency-type stuff
        // Oops I did an OOP
        float scale_factor;
        int screen_center_x;
        int screen_center_y;
} MyCam;

typedef struct {
        int x;
        int y;
} int2;

MyCam createCamera(Vector2 center, float v_size, int screenWidth, int screenHeight, float rotation);
int2 world2screen(MyCam camera, Vector2 world_pos);

// Now to basically re-implement almost all the rshapes functions

void mc_DrawRect(MyCam camera, Vector2 position, Vector2 size, Color color);

#endif