#ifndef COOKABLE_HPP
#define COOKABLE_HPP

#include <PxPhysicsAPI.h>
#include "../objects/solid_particle.hpp"
#include "../objects/mass_particle.hpp"
#include "../types/v3_f32.hpp"

struct cookable {
    objects::seconds_f64 cook_time;
    objects::solid_dynamic_multishape_particle solid;

    cookable() = default;
    cookable(objects::seconds_f64 cook_time, objects::solid_dynamic_multishape_particle &&solid);

    static cookable create_egg(
        physx::PxPhysics &physics,
        physx::PxTransform const &transform,
        physx::PxBoxGeometry const &scale_bounds,
        objects::seconds_f64 cook_time
    );

    static cookable create_steak(
        physx::PxPhysics &physics,
        physx::PxTransform const &transform,
        physx::PxBoxGeometry const &scale_bounds,
        objects::seconds_f64 cook_time
    );

    static cookable create_fries(
        physx::PxPhysics &physics,
        physx::PxTransform const &transform,
        physx::PxBoxGeometry const &scale_bounds,
        objects::seconds_f64 cook_time
    );
};

#endif