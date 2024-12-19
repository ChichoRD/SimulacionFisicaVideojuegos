#ifndef GAME_SCENE_HPP
#define GAME_SCENE_HPP

#include <list>
#include <unordered_map>
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

    objects::seconds_f64 last_delta_time;
    bool reset_pan_rotation_requested = false;
    physx::PxQuat next_pan_rotation;

    objects::solid_static_particle ground;
    pan frying_pan;
    std::list<cookable> cookables;
    std::unordered_map<physx::PxRigidActor *, std::list<cookable>::iterator> cookable_map;

    game_scene(physx::PxPhysics &physics);
    ~game_scene();

    bool init();
    bool update(objects::seconds_f64 delta_time);
    bool shutdown();
    
    cookable &add_cookable(cookable &&cookable);
    void on_cookable_pan_contact(cookable &cookable, physx::PxContactPair const &pair);
    void on_passive_mouse_motion(int x, int y);
    void on_key_press(unsigned char key, const physx::PxTransform &camera);
    void on_key_release(unsigned char key);

    void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override;
	void onWake(physx::PxActor** actors, physx::PxU32 count) override;
	void onSleep(physx::PxActor** actors, physx::PxU32 count) override;
	void onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs) override;
	void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override;
	void onAdvance(const physx::PxRigidBody* const* bodyBuffer, const physx::PxTransform* poseBuffer, const physx::PxU32 count) override;
};

#endif