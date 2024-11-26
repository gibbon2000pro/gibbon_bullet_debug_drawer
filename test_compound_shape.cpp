#include <chrono>
#include <memory>

#include "btBulletDynamicsCommon.h"
#include "gibbon_bullet_debug_drawer.h"
#include "spdlog/spdlog.h"

struct BodyContext {
    btRigidBody* mBody;
    btDefaultMotionState* mMotionState;

    BodyContext()
        : mBody(nullptr),
          mMotionState(nullptr) {
    }

    ~BodyContext() {
        if (mBody) {
            delete mBody;
            mBody = nullptr;
        }
        if (mMotionState) {
            delete mMotionState;
            mMotionState = nullptr;
        }
    }
};

std::shared_ptr<BodyContext> CreateRigidBody(
    btCollisionShape* shape,
    btScalar mass,
    const btVector3& pos,
    const btQuaternion& quat) {
    btTransform tf;
    tf.setIdentity();
    tf.setOrigin(pos);
    tf.setRotation(quat);

    auto ctx = std::make_shared<BodyContext>();
    ctx->mMotionState = new btDefaultMotionState(tf);

    btVector3 localInertia(0, 0, 0);
    if (mass > 0) {
        shape->calculateLocalInertia(mass, localInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, ctx->mMotionState, shape, localInertia);
    ctx->mBody = new btRigidBody(cInfo);
    ctx->mBody->setUserIndex(-1);

    return ctx;
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

    auto physicsWorld = std::make_shared<btDiscreteDynamicsWorld>(dispatcher.get(), broadphase.get(), solver.get(), config.get());
    physicsWorld->setGravity(btVector3(0, gravityConstant, 0));

    // ground
    auto compoundShape = std::make_shared<btCompoundShape>();

    btTransform tf;

    // ---- center cylinder
    auto centerShape = std::make_shared<btCylinderShape>(btVector3(1.0f, 0.5f, 1.0f));
    tf.setOrigin(btVector3(0, 0, 0));
    tf.setRotation(btQuaternion(0, 0, 0, 1));
    compoundShape->addChildShape(tf, centerShape.get());

    // ---- up
    auto upShape = std::make_shared<btBoxShape>(btVector3(0.5f, 0.5f, 2.0f));
    tf.setOrigin(btVector3(0.0f, 0.0f, -2.5f));
    tf.setRotation(btQuaternion(0, 0, 0, 1));
    compoundShape->addChildShape(tf, upShape.get());

    // ---- down
    tf.setOrigin(btVector3(0.0f, 0.0f, 2.5f));
    tf.setRotation(btQuaternion(0, 0, 0, 1));
    compoundShape->addChildShape(tf, upShape.get());

    // ---- left
    auto leftShape = std::make_shared<btBoxShape>(btVector3(2.0f, 0.5f, 0.5f));
    tf.setOrigin(btVector3(-2.5f, 0.0f, 0.0f));
    tf.setRotation(btQuaternion(0, 0, 0, 1));
    compoundShape->addChildShape(tf, leftShape.get());

    // ---- right
    tf.setOrigin(btVector3(2.5f, 0.0f, 0.0f));
    tf.setRotation(btQuaternion(0, 0, 0, 1));
    compoundShape->addChildShape(tf, leftShape.get());

    compoundShape->setMargin(margin);
    auto compound = CreateRigidBody(compoundShape.get(), 0.f, btVector3(0, 0, 0), btQuaternion(0, 0, 0, 1));
    compound->mBody->setFriction(friction);
    compound->mBody->setRollingFriction(rollingFriction);
    compound->mBody->setRestitution(restitution);

    physicsWorld->addRigidBody(compound->mBody);

    // cylinder ground
    /*
    auto groundShape = std::make_shared<btCylinderShape>(btVector3(5.0f, 2.5f, 5.0f));
    groundShape->setMargin(margin);
    auto groundBody = CreateRigidBody(groundShape.get(), 0.0f, btVector3(0, 0, 0), btQuaternion(0, 0, 0, 1));
    groundBody->setFriction(friction);
    groundBody->setRollingFriction(rollingFriction);
    groundBody->setRestitution(restitution);

    physicsWorld->addRigidBody(groundBody.get());
    //*/

    // ball
    auto ballShape = std::make_shared<btSphereShape>(0.2f);
    ballShape->setMargin(margin);
    auto ball = CreateRigidBody(ballShape.get(), 1.2f, btVector3(0, 4, 0), btQuaternion(0, 0, 0, 1));
    ball->mBody->setFriction(friction);
    ball->mBody->setRollingFriction(rollingFriction);
    ball->mBody->setRestitution(restitution);
    ball->mBody->setLinearVelocity(btVector3(2, 0, -0.2));
    ball->mBody->setAngularVelocity(btVector3(0, 3, 0));

    physicsWorld->addRigidBody(ball->mBody);

    // debug drawer
    auto debugDrawer = std::make_shared<::gibbon::BulletDebugDrawer>();
    debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
    physicsWorld->setDebugDrawer(debugDrawer.get());

    auto lastTime = std::chrono::system_clock::now();
    while (!debugDrawer->IsWindowClose()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(16));

        auto now = std::chrono::system_clock::now();
        float delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();
        delta = delta / 1000.0F;
        lastTime = now;

        spdlog::info("delta:{}", delta);

        physicsWorld->stepSimulation(delta);

        debugDrawer->BeginDebug();
        physicsWorld->debugDrawWorld();
        debugDrawer->EndDebug();
    }

    return 0;
}