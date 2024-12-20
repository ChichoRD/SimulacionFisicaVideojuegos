#ifndef GAME_SCENE_HPP
#define GAME_SCENE_HPP

#include <list>
#include <vector>
#include <unordered_map>
#include <PxPhysicsAPI.h>
#include "../objects/solid_particle.hpp"
#include "../objects/mass_particle.hpp"
#include "../types/v3_f32.hpp"
#include "../callbacks.hpp"
#include "../systems/particle_system.hpp"
#include "../systems/particle_generator.hpp"
#include "../generators/gravity_generator.hpp"
#include "../generators/wind_generator.hpp"
#include "../generators/spring/buoyancy_generator.hpp"

#include "pan.hpp"
#include "cookable.hpp"

extern ContactReportCallback gContactReportCallback;
extern physx::PxDefaultCpuDispatcher* gDispatcher;


struct game_scene : public physx::PxSimulationEventCallback {
    struct combined_generator {
        generators::gravity_generator gravity;
        generators::wind_generator wind;
        generators::tornado_generator tornado;
        generators::buoyancy_generator buoyancy;
        void apply_to_particles(systems::particle_system &sys, objects::seconds_f64 delta_time) {
            gravity.apply_to_particles(sys, delta_time);
            wind.apply_to_particles(sys, delta_time);
            tornado.apply_to_particles(sys, delta_time);
            buoyancy.apply_to_particles(sys, delta_time);
        }
    };

    enum particle_task {
        PARTICLE_TASK_NONE = 0,
        PAN_FRYING = 1 << 0,
        COOKABLE_COOKING = 1 << 1,
        COOKABLE_COOKED = 1 << 2,
        COOKABLE_BURNING = 1 << 3,
        COOKABLE_BURNT = 1 << 4
    };

    enum cookable_type {
        COOKABLE_TYPE_NONE = 0,
        EGG,
        STEAK,
        FRIES,

        FIRST = EGG,
        LAST = FRIES
    };

    physx::PxPhysics &physics;
    physx::PxScene *scene;

    objects::seconds_f64 time = 0.0;
    objects::seconds_f64 last_delta_time;
    bool reset_pan_rotation_requested = false;
    physx::PxQuat next_pan_rotation;
    size_t current_particle_task = PARTICLE_TASK_NONE;
    physx::PxVec3 last_cookable_contact;
    bool last_cookable_contact_valid = false;

    objects::solid_static_particle ground;
    pan frying_pan;
    std::list<cookable> cookables;
    std::vector<physx::PxRigidActor *> cookables_to_remove;
    std::unordered_map<physx::PxRigidActor *, std::list<cookable>::iterator> cookable_map;

    combined_generator pan_generator;
    systems::particle_system pan_particle_system;

    combined_generator cookable_generator;
    systems::particle_system cookables_particle_system;
    //systems::particle_system *cookables_particle_system;

    game_scene(physx::PxPhysics &physics);
    ~game_scene();

    bool init();
    bool update(objects::seconds_f64 delta_time);
    bool shutdown();
    
    cookable &add_cookable(cookable &&cookable);
    void on_cookable_pan_contact(cookable &cookable, physx::PxContactPair const &pair);
    void on_cookable_ground_contact(cookable &cookable); 

    void on_passive_mouse_motion(int x, int y);
    void on_key_press(unsigned char key, const physx::PxTransform &camera);
    void on_key_release(unsigned char key);

    void handle_particle_tasks(systems::particle_system &particle_system, size_t mask = ~0);
    void purge_particles(systems::particle_system &particle_system);
    void spawn_cookable(cookable_type type = cookable_type::COOKABLE_TYPE_NONE);
    void spawn_particle_burst(
        systems::particle_system &particle_system,
        size_t count_min, size_t count_max,
        objects::f32 size_min, objects::f32 size_max,
        objects::mass_f32 min_mass, objects::mass_f32 max_mass,
        types::v3_f32 const &colour_min, types::v3_f32 const &colour_max
    );

    void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override;
	void onWake(physx::PxActor** actors, physx::PxU32 count) override;
	void onSleep(physx::PxActor** actors, physx::PxU32 count) override;
	void onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs) override;
	void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override;
	void onAdvance(const physx::PxRigidBody* const* bodyBuffer, const physx::PxTransform* poseBuffer, const physx::PxU32 count) override;
};

#endif