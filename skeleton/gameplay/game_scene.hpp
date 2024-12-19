#ifndef GAME_SCENE_HPP
#define GAME_SCENE_HPP

#include <PxPhysicsAPI.h>
#include "../objects/solid_particle.hpp"
#include "../objects/mass_particle.hpp"
#include "../types/v3_f32.hpp"
#include "../callbacks.hpp"

#include "pan.hpp"
#include "cookable.hpp"

extern ContactReportCallback gContactReportCallback;
extern physx::PxDefaultCpuDispatcher* gDispatcher;

struct game_scene : public physx::PxSimulationEventCallback {

    physx::PxPhysics &physics;
    physx::PxScene *scene;

    objects::solid_static_particle ground;
    pan frying_pan;
    cookable egg;
    cookable steak;
    cookable fries;

    game_scene(physx::PxPhysics &physics);
    ~game_scene();

    bool init();
    bool update(objects::seconds_f64 delta_time);
    bool shutdown();
    
    void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override;
	void onWake(physx::PxActor** actors, physx::PxU32 count) override;
	void onSleep(physx::PxActor** actors, physx::PxU32 count) override;
	void onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs) override;
	void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override;
	void onAdvance(const physx::PxRigidBody* const* bodyBuffer, const physx::PxTransform* poseBuffer, const physx::PxU32 count) override;
};

#endif