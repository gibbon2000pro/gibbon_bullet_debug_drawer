#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <unordered_map>

#include "btBulletDynamicsCommon.h"
#include "gibbon_bullet_debug_drawer.h"
#include "skill_cooldown.h"

struct WorldConst {
    static constexpr float gravityConstant = -9.8F;
    static constexpr float margin = 0.05F;
    static constexpr float friction = 0.4F;
    static constexpr float rollingFriction = 0.05F;
    static constexpr float restitution = 0.4F;

    static constexpr float groundRadius = 8.0F;
    static constexpr float groundHeight = 1.5F;
    static constexpr float groundMass = 0.0F;
    static constexpr float groundPos[3] = {0, 0, 0};
    static constexpr float groundQuat[4] = {0, 0, 0, 1};

    static constexpr float ballRadius = 0.6F;
    static constexpr float ballMass = 1.2F;

    static constexpr float initMaxRadius = 2.0F;
};

btVector3 RandomInitPos() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> radius(0.0F, WorldConst::initMaxRadius);
    static std::uniform_real_distribution<float> theta(0.0F, M_PI);
    static std::uniform_real_distribution<float> height(10.0F, 20.0F);

    float r = radius(gen);
    float t = theta(gen);
    return btVector3(r * cos(t), height(gen), r * sin(t));
}

btQuaternion RandomQuat() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dis(-1.0F, 1.0F);

    return btQuaternion(dis(gen), dis(gen), dis(gen), dis(gen));
}

struct CylinderContext {
    btCylinderShape* mCylinderShape = nullptr;
    btRigidBody* mCylinderBody = nullptr;

    virtual ~CylinderContext() {
        if (mCylinderShape) {
            delete mCylinderShape;
            mCylinderShape = nullptr;
        }
        if (mCylinderBody) {
            delete mCylinderBody;
            mCylinderBody = nullptr;
        }
    }
};

struct BallContext {
    btSphereShape* mBallShape = nullptr;
    btRigidBody* mBallBody = nullptr;

    virtual ~BallContext() {
        if (mBallShape) {
            delete mBallShape;
            mBallShape = nullptr;
        }
        if (mBallBody) {
            delete mBallBody;
            mBallBody = nullptr;
        }
    }
};

struct ObjectContext {
    btCollisionShape* mShape;
    btRigidBody* mBody;
    btMotionState* mMotionState;

    ObjectContext()
        : mShape(nullptr),
          mBody(nullptr),
          mMotionState(nullptr) {
    }

    ~ObjectContext() {
        if (mShape) {
            delete mShape;
            mShape = nullptr;
        }
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

struct WorldContext {
    btDefaultCollisionConfiguration* mConfig = nullptr;
    btCollisionDispatcher* mDispatcher = nullptr;
    btDbvtBroadphase* mBroadphase = nullptr;
    btSequentialImpulseConstraintSolver* mSolver = nullptr;

    btDiscreteDynamicsWorld* mPhysicsWorld = nullptr;

    // std::shared_ptr<btCylinderShape> mGroundShape = nullptr;
    ObjectContext mGround;

    std::unordered_map<int, std::shared_ptr<ObjectContext>> mBalls;
    std::unordered_map<int, std::shared_ptr<ObjectContext>> mCylinders;

    ~WorldContext() {
        if (mPhysicsWorld) {
            delete mPhysicsWorld;
            mPhysicsWorld = nullptr;
        }
        if (mSolver) {
            delete mSolver;
            mSolver = nullptr;
        }
        if (mBroadphase) {
            delete mBroadphase;
            mBroadphase = nullptr;
        }
        if (mDispatcher) {
            delete mDispatcher;
            mDispatcher = nullptr;
        }
        if (mConfig) {
            delete mConfig;
            mConfig = nullptr;
        }
    }
};

static btRigidBody* CreateRigidBody(
    ObjectContext& object,
    btScalar mass,
    const btVector3& pos,
    const btQuaternion& quat) {
    btTransform tf;
    tf.setIdentity();
    tf.setOrigin(pos);
    tf.setRotation(quat);
    object.mMotionState = new btDefaultMotionState(tf);

    btVector3 localInertia(0, 0, 0);
    if (mass > 0) {
        object.mShape->calculateLocalInertia(mass, localInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, object.mMotionState, object.mShape, localInertia);
    object.mBody = new btRigidBody(cInfo);

    object.mBody->setUserIndex(-1);
    return object.mBody;
}

static void InitWorld(WorldContext& context) {
    context.mConfig = new btDefaultCollisionConfiguration();
    context.mDispatcher = new btCollisionDispatcher(context.mConfig);
    context.mBroadphase = new btDbvtBroadphase();
    context.mSolver = new btSequentialImpulseConstraintSolver();

    context.mPhysicsWorld = new btDiscreteDynamicsWorld(
        context.mDispatcher,
        context.mBroadphase,
        context.mSolver,
        context.mConfig);

    context.mPhysicsWorld->setGravity(btVector3(0, WorldConst::gravityConstant, 0));
}

static void InitGround(WorldContext& context) {
    context.mGround.mShape = new btConeShape(WorldConst::groundRadius, WorldConst::groundHeight);
    context.mGround.mShape->setMargin(WorldConst::margin);
    CreateRigidBody(context.mGround,
                    WorldConst::groundMass,
                    btVector3(WorldConst::groundPos[0], WorldConst::groundPos[1], WorldConst::groundPos[2]),
                    btQuaternion(WorldConst::groundQuat[0], WorldConst::groundQuat[1], WorldConst::groundQuat[2], WorldConst::groundQuat[3]));
    context.mGround.mBody->setFriction(WorldConst::friction);
    context.mGround.mBody->setRollingFriction(WorldConst::rollingFriction);
    context.mGround.mBody->setRestitution(WorldConst::restitution);

    context.mPhysicsWorld->addRigidBody(context.mGround.mBody);
}

static void CreateBall(std::shared_ptr<ObjectContext> ball, float radius = WorldConst::ballRadius, float mass = WorldConst::ballMass) {
    ball->mShape = new btSphereShape(radius);
    ball->mShape->setMargin(WorldConst::margin);
    CreateRigidBody(*ball,
                    mass,
                    RandomInitPos(),
                    RandomQuat());
    ball->mBody->setFriction(WorldConst::friction);
    ball->mBody->setRollingFriction(WorldConst::rollingFriction);
    ball->mBody->setRestitution(WorldConst::restitution);
}

static std::shared_ptr<ObjectContext> InitBall(WorldContext& context, int& num) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dis(WorldConst::ballRadius * 1.5, WorldConst::ballRadius * 3.0);

    for (int i = 20; i <= 50; ++i) {
        if (context.mBalls.find(i) == context.mBalls.end()) {
            num = i;
            auto ball = std::make_shared<ObjectContext>();
            CreateBall(ball, dis(gen), WorldConst::ballMass * 5.0F);
            context.mBalls.insert({num, ball});
            context.mPhysicsWorld->addRigidBody(ball->mBody);
            return ball;
        }
    }
    return nullptr;
}

static std::shared_ptr<ObjectContext> InitCylinder(WorldContext& context, int& num) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> r(WorldConst::ballRadius, WorldConst::ballRadius * 2.5);
    static std::uniform_real_distribution<float> h(WorldConst::ballRadius, WorldConst::ballRadius * 5.0);

    for (int i = 60; i <= 80; ++i) {
        if (context.mCylinders.find(i) == context.mCylinders.end()) {
            num = i;
            auto cylinder = std::make_shared<ObjectContext>();

            float radius = r(gen);
            float height = h(gen);

            cylinder->mShape = new btCylinderShape(btVector3(radius, height, radius));
            cylinder->mShape->setMargin(WorldConst::margin);
            CreateRigidBody(*cylinder,
                            WorldConst::ballMass * 8.0F,
                            RandomInitPos(),
                            RandomQuat());
            cylinder->mBody->setFriction(WorldConst::friction);
            cylinder->mBody->setRollingFriction(WorldConst::rollingFriction);
            cylinder->mBody->setRestitution(WorldConst::restitution);

            context.mCylinders.insert({num, cylinder});
            context.mPhysicsWorld->addRigidBody(cylinder->mBody);
            return cylinder;
        }
    }
    return nullptr;
}

int main() {
    WorldContext context;
    InitWorld(context);
    InitGround(context);

    // debug drawer
    ::gibbon::BulletDebugDrawer debugDrawer;
    debugDrawer.setDebugMode(btIDebugDraw::DBG_DrawWireframe);
    context.mPhysicsWorld->setDebugDrawer(&debugDrawer);

    ::gibbon::SkillCooldown ballNPC(5300);
    ::gibbon::SkillCooldown cylinderNPC(6700);

    auto frameTP = std::chrono::system_clock::now();
    while (!debugDrawer.IsWindowClose()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(16));

        auto now = std::chrono::system_clock::now();
        int deltaMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - frameTP).count();
        float deltaS = float(deltaMs) / 1000.0F;
        frameTP = now;

        context.mPhysicsWorld->stepSimulation(deltaS);

        // Logger::Info("delta:{} player_count:{} ball_count:{}", deltaMs, context.mPlayers.size(), context.mBalls.size());

        {
            if (ballNPC.Skill()) {
                int npcNum;
                InitBall(context, npcNum);
            }
            for (auto iter = context.mBalls.begin(); iter != context.mBalls.end();) {
                btMotionState* motionState = iter->second->mBody->getMotionState();
                if (motionState) {
                    btTransform transform;
                    motionState->getWorldTransform(transform);
                    if (transform.getOrigin().getY() < -30.f) {
                        context.mPhysicsWorld->removeRigidBody(iter->second->mBody);
                        iter = context.mBalls.erase(iter);
                        continue;
                    }
                }
                ++iter;
            }
        }

        {
            if (cylinderNPC.Skill()) {
                int npcNum;
                InitCylinder(context, npcNum);
            }
            for (auto iter = context.mCylinders.begin(); iter != context.mCylinders.end();) {
                btMotionState* motionState = iter->second->mBody->getMotionState();
                if (motionState) {
                    btTransform transform;
                    motionState->getWorldTransform(transform);
                    if (transform.getOrigin().getY() < -30.f) {
                        context.mPhysicsWorld->removeRigidBody(iter->second->mBody);
                        iter = context.mCylinders.erase(iter);
                        continue;
                    }
                }
                ++iter;
            }
        }

        debugDrawer.BeginDebug();
        context.mPhysicsWorld->debugDrawWorld();
        debugDrawer.EndDebug();
    }

    return 0;
}
