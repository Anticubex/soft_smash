#include "mycam.h"
#include <core.h>
#include <physics.h>
#include <raylib.h>
#include <raymath.h>

int main() {
        const int screenWidth = 800;
        const int screenHeight = 600;
        InitWindow(screenWidth, screenHeight, "Test Platformer");
        SetTargetFPS(60);

        Vector2 gravity = {0, 20};

        MyCam camera = createCamera((Vector2){0, 0}, 10.0, screenWidth, screenHeight, 0.0);

        Transform2D A_T = {0, 0, 0};
        Vector2 A_dim = (Vector2){1, 1};
        Collider A_C = initBoxCollider((Vector2){0, 0}, A_dim, 0);
        Transform2D B_T = {0, 0, 0};
        Vector2 B_dim = (Vector2){1.5, .5};
        Collider B_C = initBoxCollider((Vector2){0, 0}, B_dim, 0);

        while (!WindowShouldClose()) {
                BeginDrawing();
                ClearBackground(RAYWHITE);

                float dt = GetFrameTime();

                // Do input stuff
                A_T.position.x += dt * (float)(IsKeyDown(KEY_D) - IsKeyDown(KEY_A));
                A_T.position.y += dt * (float)(IsKeyDown(KEY_S) - IsKeyDown(KEY_W));
                A_T.rotation += dt * (float)(IsKeyDown(KEY_E) - IsKeyDown(KEY_Q));

                B_T.position.x += dt * (float)(IsKeyDown(KEY_L) - IsKeyDown(KEY_J));
                B_T.position.y += dt * (float)(IsKeyDown(KEY_K) - IsKeyDown(KEY_I));
                B_T.rotation += dt * (float)(IsKeyDown(KEY_O) - IsKeyDown(KEY_U));

                BeginMode2D(camera.raylib_cam);
                /* Draw Stuff Here */

                EndMode2D();

                DrawText("It Works!", 20, 20, 20, BLACK);
                EndDrawing();
        }
        CloseWindow();
        return 0;
}
