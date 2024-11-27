#ifndef _GIBBON_BULLET_DEBUG_DRAWER_H_
#define _GIBBON_BULLET_DEBUG_DRAWER_H_

#include <cmath>

#include "bullet/LinearMath/btIDebugDraw.h"
#include "raylib.h"
#include "spdlog/spdlog.h"

namespace gibbon {

inline Color ToColor(const btVector3& color) {
    return Color{
        .r = (unsigned char)std::round(color.x() * 255.f),
        .g = (unsigned char)std::round(color.y() * 255.f),
        .b = (unsigned char)std::round(color.z() * 255.f),
        .a = 255};
}

inline Vector3 ToVector3(const btVector3& vec3) {
    return Vector3{
        .x = vec3.x(),
        .y = vec3.y(),
        .z = vec3.z()};
}

class BulletDebugDrawer : public btIDebugDraw {
public:
    BulletDebugDrawer() : btIDebugDraw() {
        const int screenWidth = 800;
        const int screenHeight = 450;
        InitWindow(screenWidth, screenHeight, "gibbon bullet debug drawer");

        mCamera = {
            .position = {-4.f, 18.f, 10.f},
            .target = {0.0f, 0.0f, 0.0f},
            .up = {0.0f, 1.0f, 0.0f},
            .fovy = 45.0f,
            .projection = 0};
    }

    ~BulletDebugDrawer() {
        CloseWindow();
    }

    bool IsWindowClose() {
        return WindowShouldClose();
    }

    void BeginDebug() {
        BeginDrawing();
        ClearBackground(SKYBLUE);

        if (IsKeyDown(KEY_RIGHT))
            mCamera.position.x += 0.2f;
        else if (IsKeyDown(KEY_LEFT))
            mCamera.position.x -= 0.2f;
        else if (IsKeyDown(KEY_DOWN))
            mCamera.position.y -= 0.2f;
        else if (IsKeyDown(KEY_UP))
            mCamera.position.y += 0.2f;

        BeginMode3D(mCamera);
    }

    void EndDebug() {
        EndMode3D();
        EndDrawing();
    }

    void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override {
        DrawLine3D(ToVector3(from), ToVector3(to), ToColor(color));
    }

    // void drawSphere(btScalar radius, const btTransform& transform, const btVector3& color) override {
    //     spdlog::info("drawSphere()");
    //     DrawSphere(ToVector3(transform.getOrigin()), radius, ToColor(color));
    // }

    void drawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2, const btVector3& color, btScalar /*alpha*/) override {
        DrawTriangle3D(ToVector3(v0), ToVector3(v1), ToVector3(v2), ToColor(color));
    }

    void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) override {
    }

    void reportErrorWarning(const char* warningString) override {
        spdlog::warn("{}", warningString);
    }

    void draw3dText(const btVector3& location, const char* textString) override {
    }

    void setDebugMode(int debugMode) override {
        mDebugMode = debugMode;
    }

    int getDebugMode() const override {
        return mDebugMode;
    }

private:
    int mDebugMode;
    Camera mCamera;
};

}  // namespace gibbon

#endif  //_GIBBON_BULLET_DEBUG_DRAWER_H_