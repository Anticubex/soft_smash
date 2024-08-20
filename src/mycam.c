#include "mycam.h"

MyCam createCamera(Vector2 center, float v_size, int screenWidth, int screenHeight, float rotation) {
        int centerX = screenWidth / 2;
        int centerY = screenHeight / 2;
        int scale_factor = screenHeight / v_size;
        return (MyCam){
            center,
            v_size,
            screenWidth,
            screenHeight,
            rotation,
            (Camera2D){
                (Vector2){centerX, centerY},
                (Vector2){0, 0},
                rotation,
                scale_factor,
            },
            scale_factor,
            centerX,
            centerY,
        };
}

int2 world2screen(MyCam camera, Vector2 world_pos) {
        return (int2){
            camera.screen_center_x + (int)((world_pos.x - camera.center.x) * camera.scale_factor),
            camera.screen_center_y + (int)((world_pos.y - camera.center.y) * camera.scale_factor),
        };
}

void mc_DrawRect(MyCam camera, Vector2 position, Vector2 size, Color color) {
        int2 ps = world2screen(camera, position);
        DrawRectangle(ps.x, ps.y, size.x * camera.scale_factor, size.y * camera.scale_factor, color);
}
