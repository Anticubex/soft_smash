#include <mycam.hpp>

MyCam::MyCam(Vector2 center, float v_size, int screenWidth, int screenHeight, float rotation) : center(center),
                                                                                                screenWidth(screenWidth),
                                                                                                screenHeight(screenHeight),
                                                                                                rotation(rotation),
                                                                                                scale_factor(screenHeight / v_size),
                                                                                                screen_center_x(screenWidth * 0.5f),
                                                                                                screen_center_y(screenHeight * 0.5f) {}

int2 MyCam::world2screen(const Vector2 &world_pos) const {
        return (int2){
            screen_center_x + (int)((world_pos.x - center.x) * scale_factor),
            screen_center_y + (int)((world_pos.y - center.y) * scale_factor),
        };
}

void MyCam::begin() const {
        BeginMode2D((Camera2D){
            (Vector2){screen_center_x, screen_center_y},
            center,
            rotation,
            scale_factor,
        });
}

void MyCam::end() const {
        EndMode2D();
}

void MyCam::updateViewport(const int2 &dimensions, const float &v_size) {
        scale_factor = screenHeight / v_size;
        screen_center_x = screenWidth * 0.5f;
        screen_center_y = screenHeight * 0.5f;
}
