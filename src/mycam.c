#include "mycam.h"

MyCam createCamera(Vector2 center, float v_size, int screenWidth, int screenHeight, float rotation) {
        int centerX = screenWidth / 2;
        int centerY = screenHeight / 2;
        int scale_factor = screenHeight / v_size;
        return (MyCam){
            .center = center,
            .v_size = v_size,
            .screenWidth = screenWidth,
            .screenHeight = screenHeight,
            .rotation = rotation,
            .raylib_cam = (Camera2D){
                (Vector2){centerX, centerY},
                center,
                rotation,
                scale_factor,
            },
            .scale_factor = scale_factor,
            .screen_center_x = centerX,
            .screen_center_y = centerY,
        };
}

int2 world2screen(MyCam camera, Vector2 world_pos) {
        return (int2){
            camera.screen_center_x + (int)((world_pos.x - camera.center.x) * camera.scale_factor),
            camera.screen_center_y + (int)((world_pos.y - camera.center.y) * camera.scale_factor),
        };
}

// Recalculates the raylib_cam
void updateCamera(MyCam *camera) {
        int centerX = camera->screenWidth / 2;
        int centerY = camera->screenHeight / 2;
        int scale_factor = camera->screenHeight / camera->v_size;
        camera->raylib_cam = (Camera2D){
            (Vector2){centerX, centerY},
            camera->center,
            camera->rotation,
            scale_factor,
        };
        camera->scale_factor = scale_factor;
        camera->screen_center_x = centerX;
        camera->screen_center_y = centerY;
}

void mc_DrawRect(MyCam camera, Vector2 position, Vector2 size, Color color) {
        int2 ps = world2screen(camera, position);
        DrawRectangle(ps.x, ps.y, size.x * camera.scale_factor, size.y * camera.scale_factor, color);
}
