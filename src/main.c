#include <core/core.h>
#include <core/physics.h>
#include <core/render.h>
#include <debug.h>
#include <mycam.h>
#include <raylib.h>
#include <raymath.h>

/* Currently just some testing builds, no real game yet */

int main() {

        const int screenWidth = 800;
        const int screenHeight = 600;
        InitWindow(screenWidth, screenHeight, "Test Platformer");
        SetTargetFPS(60);

        WorldValues worldValues = {.gravity = {0, 1}, .airPressure = 1.0f};

        MyCam camera = createCamera((Vector2){0, 0}, 10.0, screenWidth, screenHeight, 0.0);

        while (!WindowShouldClose()) {
                BeginDrawing();
                ClearBackground(RAYWHITE);

                float dt = GetFrameTime();

                updateCamera(&camera);
                BeginMode2D(camera.raylib_cam);
                /* Draw Stuff Here */

                EndMode2D();
        }

        CloseWindow();
        return 0;
}
