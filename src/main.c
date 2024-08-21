#include "mycam.h"
#include <core.h>
#include <physics.h>
#include <raylib.h>
#include <raymath.h>

/* Currently just some testing builds, no real game yet */

int main() {
        const int screenWidth = 800;
        const int screenHeight = 600;
        InitWindow(screenWidth, screenHeight, "Test Platformer");
        SetTargetFPS(60);

        Vector2 gravity = {0, 20};

        MyCam camera = createCamera((Vector2){0, 0}, 10.0, screenWidth, screenHeight, 0.0);

        while (!WindowShouldClose()) {
                BeginDrawing();
                ClearBackground(RAYWHITE);

                float dt = GetFrameTime();

                BeginMode2D(camera.raylib_cam);
                /* Draw Stuff Here */

                EndMode2D();

                DrawText("It Works!", 20, 20, 20, BLACK);
                EndDrawing();
        }
        CloseWindow();
        return 0;
}
