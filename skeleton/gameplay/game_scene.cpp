#include "game_scene.hpp"
#include <cassert>

static bool init_game_scene_physics(
    physx::PxPhysics &physics,
    physx::PxScene *&out_scene,
    physx::PxDefaultCpuDispatcher *&out_dispatcher
) {
	physx::PxSceneDesc sceneDesc(physics.getTolerancesScale());
	sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);

	out_dispatcher = physx::PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = out_dispatcher;
	sceneDesc.filterShader = contactReportFilterShader;

	sceneDesc.simulationEventCallback = &gContactReportCallback;

	out_scene = physics.createScene(sceneDesc);
	return true;
}

game_scene::game_scene(physx::PxPhysics &physics)
    : physics(physics) {
    assert(init_game_scene_physics(physics, scene, gDispatcher));
    scene->setSimulationEventCallback(this);
}

game_scene::~game_scene() {
    scene->release();
}

bool game_scene::init() {
    ground = objects::solid_static_particle(
        physics,
        physx::PxTransform(physx::PxVec3(0.0f, -10.0f, 0.0f)),
        *physics.createMaterial(0.5f, 0.5f, 0.6f),
        physx::PxBoxGeometry(100.0f, 1.0f, 100.0f),
        physx::PxVec4(0.5f, 0.5f, 0.5f, 1.0f)
    );
    scene->addActor(*ground.rigid_static);

    frying_pan = pan(physics, physx::PxTransform(physx::PxVec3(0.0f, 5.0f, 0.0f)));
    scene->addActor(*frying_pan.pan_solid.rigid_dynamic);

    egg = cookable::create_egg(
        physics,
        physx::PxTransform(physx::PxVec3(0.0f, 10.0f, 0.0f)),
        physx::PxBoxGeometry(2.0f, 2.0f, 2.0f),
        10.0
    );
    scene->addActor(*egg.solid.rigid_dynamic);

    steak = cookable::create_steak(
        physics,
        physx::PxTransform(physx::PxVec3(0.0f, 15.0f, 0.0f)),
        physx::PxBoxGeometry(2.0f, 1.0f, 2.0f),
        10.0
    );
    scene->addActor(*steak.solid.rigid_dynamic);

    fries = cookable::create_fries(
        physics,
        physx::PxTransform(physx::PxVec3(0.0f, 20.0f, 0.0f)),
        physx::PxBoxGeometry(3.0f, 1.0f, 1.0f),
        10.0
    );
    scene->addActor(*fries.solid.rigid_dynamic);
    
    return true;
}

bool game_scene::update(objects::seconds_f64 delta_time) {
    // TODO

    auto &frying_pan_rb = frying_pan.pan_solid.rigid_dynamic;
	frying_pan_rb->setKinematicTarget(physx::PxTransform(
		frying_pan_rb->getGlobalPose().p + physx::PxVec3(0.0f, 0.0f, 1.0f) * delta_time
	));

    scene->simulate(delta_time);
	physx::PxU32 error_state = 0;
	scene->fetchResults(true, &error_state);

    return error_state == 0;
}

bool game_scene::shutdown() {
    // TODO particles
    return false;
}

void game_scene::onConstraintBreak(physx::PxConstraintInfo *constraints, physx::PxU32 count) {
    (void)constraints;
    (void)count;
}

void game_scene::onWake(physx::PxActor **actors, physx::PxU32 count) {
    (void)actors;
    (void)count;
}

void game_scene::onSleep(physx::PxActor **actors, physx::PxU32 count) {
    (void)actors;
    (void)count;
}

void game_scene::onContact(const physx::PxContactPairHeader &pairHeader, const physx::PxContactPair *pairs, physx::PxU32 nbPairs) {
    //pairHeader.pairs
}

void game_scene::onTrigger(physx::PxTriggerPair *pairs, physx::PxU32 count) {
    (void)pairs;
    (void)count;
}

void game_scene::onAdvance(const physx::PxRigidBody *const *bodyBuffer, const physx::PxTransform *poseBuffer, const physx::PxU32 count) {
    (void)bodyBuffer;
    (void)poseBuffer;
    (void)count;
}
