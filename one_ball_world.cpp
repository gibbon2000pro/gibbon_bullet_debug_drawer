#include <chrono>
#include <memory>
#include <thread>

#include "btBulletDynamicsCommon.h"
#include "gibbon_bullet_debug_drawer.h"
#include "spdlog/spdlog.h"

std::shared_ptr<btRigidBody> CreateRigidBody(
    btCollisionShape* shape,
    btScalar mass,
    const btVector3& pos,
    const btQuaternion& quat) {
    btTransform tf;
    tf.setIdentity();
    tf.setOrigin(pos);
    tf.setRotation(quat);
    btDefaultMotionState* motionState = new btDefaultMotionState(tf);

    btVector3 localInertia(0, 0, 0);
    if (mass > 0) {
        shape->calculateLocalInertia(mass, localInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, motionState, shape, localInertia);
    auto body = std::make_shared<btRigidBody>(cInfo);

    // auto body = std::make_shared<btRigidBody>(mass, motionState, shape, localInertia);

    body->setUserIndex(-1);
    return body;
}

int main() {
    constexpr float gravityConstant = -9.8F;
    constexpr float margin = 0.05F;
    constexpr float friction = 0.5F;
    constexpr float rollingFriction = 0.1F;
    constexpr float restitution = 0.6F;

    auto config = std::make_shared<btDefaultCollisionConfiguration>();
    auto dispatcher = std::make_shared<btCollisionDispatcher>(config.get());
    auto broadphase = std::make_shared<btDbvtBroadphase>();
    auto solver = std::make_shared<btSequentialImpulseConstraintSolver>();

    auto physicsWorld = std::make_shared<btDiscreteDynamicsWorld>(
        dispatcher.get(),
        broadphase.get(),
        solver.get(),
        config.get());

    physicsWorld->setGravity(btVector3(0, gravityConstant, 0));

    // ground
    const float groundRadius = 5.0F;
    const float groundHeight = 0.3F;
    const float groundMass = 0.0F;
    const btVector3 groundPos(0, 0, 0);
    const btQuaternion groundQuat(0, 0, 0, 1);

    auto groundShape = std::make_shared<btCylinderShape>(btVector3(groundRadius, groundHeight * 0.5, groundRadius));
    groundShape->setMargin(margin);
    auto groundBody = CreateRigidBody(groundShape.get(), groundMass, groundPos, groundQuat);
    groundBody->setFriction(friction);
    groundBody->setRollingFriction(rollingFriction);
    groundBody->setRestitution(restitution);

    physicsWorld->addRigidBody(groundBody.get());

    // ball
    const float ballRadius = 0.6F;
    const float ballMass = 1.2F;
    const btVector3 ballPos(-3, 4, 0);
    const btQuaternion ballQuat(0, 0, 0, 1);

    auto ballShape = std::make_shared<btSphereShape>(ballRadius);
    ballShape->setMargin(margin);
    auto ballBody = CreateRigidBody(ballShape.get(), ballMass, ballPos, ballQuat);
    ballBody->setFriction(friction);
    ballBody->setRollingFriction(rollingFriction);
    ballBody->setRestitution(restitution);
    ballBody->setLinearVelocity(btVector3(2, 0, -1));
    ballBody->setAngularVelocity(btVector3(0, 3, 0));

    physicsWorld->addRigidBody(ballBody.get());

    ::gibbon::BulletDebugDrawer* debugDrawer = new ::gibbon::BulletDebugDrawer();
    debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
    physicsWorld->setDebugDrawer(debugDrawer);

    auto lastTime = std::chrono::system_clock::now();
    while (!debugDrawer->IsWindowClose()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(16));

        auto now = std::chrono::system_clock::now();
        float delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();
        delta = delta / 1000.0F;
        lastTime = now;

        // spdlog::info("delta:{}", delta);

        physicsWorld->stepSimulation(delta);

        {
            // auto ballFramePos = ballBody->getWorldTransform().getOrigin();
            // auto ballFrameQuat = ballBody->getWorldTransform().getRotation();
            // ::spdlog::info("ball_pos:(x:{},y:{},z:{}) ball_quat:(x:{},y:{},z:{},w:{})",
            //                ballFramePos.getX(), ballFramePos.getY(), ballFramePos.getZ(),
            //                ballFrameQuat.getX(), ballFrameQuat.getY(), ballFrameQuat.getZ(), ballFrameQuat.getW());
        }

        debugDrawer->BeginDebug();
        physicsWorld->debugDrawWorld();
        debugDrawer->EndDebug();
    }

    return 0;
}