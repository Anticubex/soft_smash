#pragma once

#include <raylib.h>
#include <raymath.h>

typedef struct int2 {
        int x;
        int y;
} int2;

class MyCam {
      public:
        MyCam(Vector2 center, float v_size, int screenWidth, int screenHeight, float rotation);
        int2 world2screen(const Vector2 &worldPos) const;
        Vector2 screen2world(const Vector2 &screenPos) const;

        void begin() const;
        void end() const;

        Vector2 center;
        float rotation;

        void updateViewport(const int2 &dimensions, const float &v_size);

      private:
        // The size in world space (vertically only, to preserve aspect ratio)
        float v_size;
        int screenWidth;
        int screenHeight;

        float scale_factor;
        float screen_center_x;
        float screen_center_y;
};
