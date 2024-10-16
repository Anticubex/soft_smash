// Made by Teo Fontana (c) 2024

#include <core/collision.h>
#include <core/core.h>
#include <core/physics.h>
#include <core/render.h>
#include <debug.h>
#include <mycam.h>
#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>

/* Currently just some testing builds, no real game yet */

int main() {
        const int screenWidth = 800;
        const int screenHeight = 600;
        InitWindow(screenWidth, screenHeight, "Test Platformer");
        SetTargetFPS(60);

        WorldValues worldValues = {.gravity = {0, 0}, .airPressure = 1.0f};

        MyCam camera = createCamera((Vector2){0, 0}, 10.0, screenWidth, screenHeight, 0.0);

        SoftBody body1 = createEmptySoftBody(
            (SoftBodyType_Springs) | (SoftBodyType_Pressure) | (SoftBodyType_Shape), // type
            1.0f,                                                                    // mass
            0.1f,                                                                    // drag
            100.f,                                                                   // spring strength
            5.f,                                                                     // spring dampening
            10.f,                                                                    // shape spring strength
            25.f                                                                     // nRT
        );
        // // circleSoftbody(&body1, (Vector2){-3.f, 0.f}, 2.f, 15);
        rectSoftbody(&body1, (Vector2){-5.0, -1.5}, (Vector2){5.0, 3.0}, 5, 3, true);

        SoftBody body2 = createEmptySoftBody(
            (SoftBodyType_Springs) | (SoftBodyType_Pressure) | (SoftBodyType_Shape), // type
            1.0f,                                                                    // mass
            0.1f,                                                                    // drag
            100.f,                                                                   // spring strength
            5.f,                                                                     // spring dampening
            10.f,                                                                    // shape spring strength
            25.f                                                                     // nRT
        );
        circleSoftbody(&body2, (Vector2){3.f, 0.f}, 2.f, 15);
        // // rectSoftbody(&body2, (Vector2){5.0, -1.5}, (Vector2){5.0, 3.0}, 5, 3, true);

        // int b1pts = 3;
        // body1.numPoints = b1pts;
        // body1.pointPos = malloc(sizeof(Vector2) * b1pts);
        // body1.shape = calloc(b1pts, sizeof(Vector2));
        // body1.pointPos[0] = (Vector2){-5.0f, -1.5f};
        // body1.pointPos[1] = (Vector2){-5.0f, 1.5f};
        // body1.pointPos[2] = (Vector2){-6.0f, 0.0f};
        // body1.pointVel = calloc(b1pts, sizeof(Vector2));
        // body1.numSprings = 1;
        // body1.springA = malloc(sizeof(int));
        // body1.springB = malloc(sizeof(int));
        // body1.lengths = malloc(sizeof(float));
        // body1.springA[0] = 0;
        // body1.springB[0] = 1;
        // body1.lengths[0] = 3.0f;
        // body1.numSurfaces = 1;
        // body1.surfaceA = malloc(sizeof(int));
        // body1.surfaceB = malloc(sizeof(int));
        // body1.surfaceA[0] = 0;
        // body1.surfaceB[0] = 1;

        // body2.numPoints = 2;
        // body2.pointPos = malloc(sizeof(Vector2) * 2);
        // body2.shape = malloc(sizeof(Vector2) * 2);
        // body2.pointPos[0] = (Vector2){1.5f, -0.5f};
        // body2.pointPos[1] = (Vector2){3.5f, 0.5f};
        // body2.shape[0] = (Vector2){-1.0f, 0.0f};
        // body2.shape[1] = (Vector2){1.0f, 0.0f};
        // body2.pointVel = calloc(2, sizeof(Vector2));
        // body2.numSprings = 1;
        // body2.springA = malloc(sizeof(int));
        // body2.springB = malloc(sizeof(int));
        // body2.lengths = malloc(sizeof(float));
        // body2.springA[0] = 0;
        // body2.springB[0] = 1;
        // body2.lengths[0] = 2.0f;
        // body2.numSurfaces = 1;
        // body2.surfaceA = malloc(sizeof(int));
        // body2.surfaceB = malloc(sizeof(int));
        // body2.surfaceA[0] = 0;
        // body2.surfaceB[0] = 1;

        applyImpulse(&body1, (Vector2){1.f, 0.f});
        applyImpulse(&body2, (Vector2){-1.f, 0.f});

        SoftBodyRenderer rend1 = {.fillColor = RED, .borderColor = BLACK, .thickness = 0.1f};
        autogenerateRendererFromSurface(body1, &rend1);

        SoftBodyRenderer rend2 = {.fillColor = BLUE, .borderColor = BLACK, .thickness = 0.1f};
        autogenerateRendererFromSurface(body2, &rend2);

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

                if (testspeedmultiplier != 0.0f) {
                        update_SoftBody(&body1, worldValues, dt * testspeedmultiplier);
                        update_SoftBody(&body2, worldValues, dt * testspeedmultiplier);
                } else if (IsKeyPressed(KEY_SPACE)) {
                        update_SoftBody(&body1, worldValues, dt);
                        update_SoftBody(&body2, worldValues, dt);
                } else
                        goto skipCollision;

                CollisionData data = checkCollision(body1, body2);
                if (data.collided) {
                        if (data.invert) {
                                handleCollision(body2, body1, data, SoftBodyMaterial_DEFAULT, SoftBodyMaterial_DEFAULT, dt);
                        } else {
                                handleCollision(body1, body2, data, SoftBodyMaterial_DEFAULT, SoftBodyMaterial_DEFAULT, dt);
                        }
                }
        skipCollision:
                // camera.center = body1.shapePosition;
                updateCamera(&camera);

                BeginMode2D(camera.raylib_cam);
                /* Draw Stuff Here */
                // renderSoftbody(body1, rend1);
                // renderSoftbody(body2, rend2);
                DrawSoftbody_debug(body1);
                DrawSoftbody_debug(body2);

                if (data.collided) {
                        SoftBody *a, *b;
                        if (data.invert) {
                                a = &body2;
                                b = &body1;
                        } else {
                                a = &body1;
                                b = &body2;
                        }
                        DrawCircleV(a->pointPos[data.point], 0.2f, RED);
                        DrawLineEx(b->pointPos[b->surfaceA[data.edge]], b->pointPos[b->surfaceB[data.edge]], 0.1f, RED);
                }

                EndMode2D();

                float V = 0.f;
                for (int i = 0; i < body1.numSurfaces; i++) {
                        Vector2 a = body1.pointPos[body1.surfaceA[i]];
                        Vector2 b = body1.pointPos[body1.surfaceB[i]];
                        V += a.x * b.y - a.y * b.x;
                }
                V *= 0.5f;

                // DrawText(TextFormat("%f", body1.bounds.max.x), 20, 20, 20, BLACK);
                DrawText(TextFormat("Simulation Speed %0.1fx", testspeedmultiplier), 20, 40, 20, BLACK);
                EndDrawing();
        }

        freeSoftbody(&body1);
        freeRenderer(&rend1);
        freeSoftbody(&body2);
        freeRenderer(&rend2);
        CloseWindow();
        return 0;
}