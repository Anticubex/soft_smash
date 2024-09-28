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

        SoftBody body1 = createEmptySoftBody(
            SoftBodyType_Springs | SoftBodyType_Pressure | SoftBodyType_Shape, // type
            1.f,                                                               // mass
            1.f,                                                               // drag
            20.f,                                                              // spring strength
            2.f,                                                               // spring dampening
            5.f,                                                               // shape spring strength
            10.f                                                               // nRT
        );
        rectSoftbody(&body1, (Vector2){-2.5, -1.5}, (Vector2){5.0, 3.0}, 5, 3, true);

        // SoftBody body1 = createEmptySoftBody(
        //     (SoftBodyType_Springs) | (SoftBodyType_Pressure) | (0 & SoftBodyType_Shape), // type
        //     1.0f,                                                                        // mass
        //     6.f,                                                                         // drag
        //     25.f,                                                                        // spring strength
        //     5.f,                                                                         // spring dampening
        //     0.1f,                                                                        // shape spring strength
        //     25.f                                                                         // nRT
        // );
        // circleSoftbody(&body1, (Vector2){-3.0, 0}, 2.0, 15);

        SoftBodyRenderer rend1 = {.fillColor = RED, .borderColor = BLACK, .thickness = 0.1f};
        autogenerateRendererFromSurface(body1, &rend1);

        // SoftBody body2 = createEmptySoftBody(SoftBodyType_Pressure, 10, 1, 10, 1, 1, 10);
        // circleSoftbody(&body2, (Vector2){3.0, 0}, 2.0, 10);

        while (!WindowShouldClose()) {
                BeginDrawing();
                ClearBackground(RAYWHITE);

                float dt = GetFrameTime();

                // testspeedmultiplier
                float testspeedmultiplier =
                    IsKeyDown(KEY_ZERO)    ? 1.0f
                    : IsKeyDown(KEY_NINE)  ? .9f
                    : IsKeyDown(KEY_EIGHT) ? .8f
                    : IsKeyDown(KEY_SEVEN) ? .7f
                    : IsKeyDown(KEY_SIX)   ? .6f
                    : IsKeyDown(KEY_FIVE)  ? .5f
                    : IsKeyDown(KEY_FOUR)  ? .4f
                    : IsKeyDown(KEY_THREE) ? .3f
                    : IsKeyDown(KEY_TWO)   ? .2f
                    : IsKeyDown(KEY_ONE)   ? .1f
                                           : 0.0f;

                if (testspeedmultiplier != 0.0f)
                        update_SoftBody(&body1, worldValues, dt * testspeedmultiplier);
                if (IsKeyPressed(KEY_SPACE))
                        update_SoftBody(&body1, worldValues, dt);
                // update_SoftBody(&body2, worldValues, dt);

                camera.center = body1.shapePosition;
                updateCamera(&camera);

                BeginMode2D(camera.raylib_cam);
                /* Draw Stuff Here */
                // DrawSoftbody_debug(body1);
                renderSoftbody(body1, rend1);

                EndMode2D();

                float V = 0.f;
                for (int i = 0; i < body1.numSurfaces; i++) {
                        Vector2 a = body1.pointPos[body1.surfaceA[i]];
                        Vector2 b = body1.pointPos[body1.surfaceB[i]];
                        V += a.x * b.y - a.y * b.x;
                }
                V *= 0.5f;

                DrawText(TextFormat("%.4f, %.4f", body1.shapePosition.x, body1.shapePosition.y), 20, 20, 20, BLACK);
                DrawText(TextFormat("Simulation Speed %0.1fx", testspeedmultiplier), 20, 40, 20, BLACK);
                DrawText(TextFormat("Volume: %f", V), 20, 60, 20, BLACK);
                EndDrawing();
        }

        freeSoftbody(&body1);
        // freeSoftbody(&body2);
        CloseWindow();
        return 0;
}